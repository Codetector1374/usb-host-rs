use alloc::sync::Arc;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::marker::PhantomData;
use core::time::Duration;

use spin::RwLock;

use crate::{UsbHAL, USBErrorKind, USBResult, as_slice, as_mut_slice};
use crate::descriptor::USBInterfaceDescriptorSet;
use crate::structs::{USBDevice, USBPipe};
use crate::items::{EndpointType, TransferBuffer};

pub struct MassStorageDriver<H: UsbHAL> {
    __phantom: PhantomData<H>,
}

#[repr(C, packed)]
#[derive(Debug)]
struct CommandBlockWrapper {
    signature: u32,
    tag: u32,
    transfer_length: u32,
    flags: u8,
    lun: u8,
    command_len: u8,
    command: [u8; 16],
}

impl CommandBlockWrapper {
    const SIGNATURE: u32 = 0x43425355;
    const FLAG_IN: u8 = 0x80;
    const FLAG_OUT: u8 = 0x00;

    pub fn new(tag: u32, transfer_length: u32, flags: u8, lun: u8, command: &[u8]) -> Self {
        assert!(command.len() >= 1);
        assert!(command.len() <= 16);
        let mut command_ = [0u8; 16];
        (&mut command_[..command.len()]).copy_from_slice(command);

        CommandBlockWrapper {
            signature: Self::SIGNATURE,
            tag,
            transfer_length,
            flags,
            lun,
            command_len: command.len() as u8,
            command: command_,
        }
    }
}

const_assert_size!(CommandBlockWrapper, 31);

#[repr(C, packed)]
#[derive(Debug, Default)]
struct CommandStatusWrapper {
    signature: u32,
    tag: u32,
    data_residue: u32,
    status: u8,
}

const_assert_size!(CommandStatusWrapper, 13);

impl CommandStatusWrapper {
    const SIGNATURE: u32 = 0x53425355;
}


struct BulkOnlyProtocol {
    input_pipe: Arc<RwLock<USBPipe>>,
    output_pipe: Arc<RwLock<USBPipe>>,
}

impl BulkOnlyProtocol {
    fn transfer(&self, lun: u8, command: &[u8], buffer: TransferBuffer) -> USBResult<usize> {
        let flags = match &buffer {
            TransferBuffer::Read(_) => CommandBlockWrapper::FLAG_IN,
            TransferBuffer::Write(_) => CommandBlockWrapper::FLAG_OUT,
            // TODO
            TransferBuffer::None => CommandBlockWrapper::FLAG_OUT,
        };

        let input_lock = self.input_pipe.read();
        let output_lock = self.output_pipe.read();

        let len = buffer.len();
        let cbw = CommandBlockWrapper::new(0, len as u32, flags, lun, command);
        output_lock.bulk_write(as_slice(&cbw))?;

        if let TransferBuffer::Write(slice) = buffer {
            output_lock.bulk_write(slice)?;
        } else if let TransferBuffer::Read(slice) = buffer {
            input_lock.bulk_read(slice)?;
        }

        let mut csw = CommandStatusWrapper::default();
        input_lock.bulk_read(as_mut_slice(&mut csw))?;

        assert_eq!(csw.signature, CommandStatusWrapper::SIGNATURE);
        if csw.data_residue != 0 || csw.status != 0 {
            warn!("got a bad CSW: {:?}", csw);
        }

        Ok(len)
    }
}

pub trait SimpleBlockDevice {
    /// must be divisible by 512.
    fn sector_size(&self) -> u64;

    /// buf must be divisible by sector_size()
    fn read_sector(&mut self, n: u64, buf: &mut [u8]) -> USBResult<usize>;

    /// buf must be divisible by sector_size()
    fn write_sector(&mut self, n: u64, buf: &[u8]) -> USBResult<usize>;
}

#[derive(Debug, Clone, Copy)]
enum LbaSize {
    Size10,
    Size16,
}

#[repr(C)]
#[derive(Default)]
struct InquiryResponse {
    flags: [u8; 8],
    vendor_info: [u8; 8],
    product_info: [u8; 16],
    product_revision: [u8; 4],
}

const_assert_size!(InquiryResponse, 36);

impl InquiryResponse {
    fn vendor_name(&self) -> String {
        String::from_utf8_lossy(self.vendor_info.as_ref()).to_string()
    }

    fn product_name(&self) -> String {
        String::from_utf8_lossy(self.product_info.as_ref()).to_string()
    }

    fn revision(&self) -> String {
        String::from_utf8_lossy(self.product_revision.as_ref()).to_string()
    }
}

#[derive(Debug, Clone, Copy)]
enum FormatType {
    UnformattedDisk,
    FormattedDisk,
    NoDisk,
}

#[derive(Debug, Clone)]
struct FormatCapacity {
    block_count: u32,
    block_length: u32,
    format: FormatType,
}

#[derive(Debug, Clone)]
struct Capacity {
    block_count: u64,
    block_length: u32,
    lba_size: LbaSize,
}

struct SCSIBuilder(Vec<u8>);

impl SCSIBuilder {
    pub fn new(command: u8) -> Self {
        let mut b = SCSIBuilder(Vec::new());
        b.0.reserve_exact(16);
        b.0.push(command);
        b
    }
    pub fn repeat_zeros(mut self, num: usize) -> Self {
        for _ in 0..num {
            self.0.push(0);
        }
        self
    }
    pub fn push_bytes(mut self, buf: &[u8]) -> Self {
        for b in buf.iter() {
            self.0.push(*b)
        }
        self
    }
    pub fn push_u8(mut self, num: u8) -> Self {
        self.0.push(num);
        self
    }
    pub fn push_u16(mut self, num: u16) -> Self {
        self.push_bytes(num.to_be_bytes().as_ref())
    }
    pub fn push_u32(mut self, num: u32) -> Self {
        self.push_bytes(num.to_be_bytes().as_ref())
    }
    pub fn push_u64(mut self, num: u64) -> Self {
        self.push_bytes(num.to_be_bytes().as_ref())
    }
    pub fn into_vec(self) -> Vec<u8> {
        self.0
    }
    pub fn as_slice(&self) -> &[u8] {
        self.0.as_slice()
    }
}

struct TransparentSCSI {
    bulk: BulkOnlyProtocol,
    inquiry: InquiryResponse,
    format_capacities: Vec<FormatCapacity>,
    capacity: Capacity,

}

fn read_u32(buf: &[u8], start: usize) -> u32 {
    u32::from_be_bytes([buf[start], buf[start+1], buf[start+2], buf[start+3]])
}

fn read_u64(buf: &[u8], start: usize) -> u64 {
    u64::from_be_bytes([
        buf[start], buf[start+1], buf[start+2], buf[start+3],
        buf[start+4], buf[start+5], buf[start+6], buf[start+7],
    ])
}

impl TransparentSCSI {
    pub fn new(bulk: BulkOnlyProtocol) -> USBResult<Self> {

        let mut inquiry = InquiryResponse::default();
        bulk.transfer(0, &[0x12, 0x00, 0x00, 0x00, 0x24, 0x00], TransferBuffer::Read(as_mut_slice(&mut inquiry)))?;

        info!("Device Info:\nMfg: {}\nPrd: {}\nRev: {}", inquiry.vendor_name(), inquiry.product_name(), inquiry.revision());

        let format_capacities = Self::parse_format_capacities(&bulk)?;

        let capacity = Self::read_capacity(&bulk)?;
        if capacity.block_length % 512 != 0 {
            return USBErrorKind::InvalidState.err("device reports a block length not divisible by 512");
        }

        info!("Got thing with capacity: {:?}", capacity);

        Ok(Self {
            bulk,
            inquiry,
            format_capacities,
            capacity,
        })
    }

    fn parse_format_capacities(bulk: &BulkOnlyProtocol) -> USBResult<Vec<FormatCapacity>> {
        let mut slice_array = [0u8; 252];
        bulk.transfer(0, &[0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00], TransferBuffer::Read(&mut slice_array))?;
        let slice = slice_array.as_ref();

        let capacity_count = read_u32(slice, 0) / 8;
        debug!("capacity count: {}", capacity_count);

        if capacity_count > 62 {
            return USBErrorKind::InvalidDescriptor.err("device reports more than 62 entries from READ FORMAT CAPACITIES");
        }

        let mut capacities: Vec<FormatCapacity> = Vec::new();

        for i in 0..capacity_count as usize {
            let rec_start = 4 + 8 * i;

            let block_count = read_u32(slice, rec_start);
            let block_length = read_u32(slice, rec_start + 5);
            let format = match slice[rec_start+4] & 0b11 {
                0b01 => FormatType::UnformattedDisk,
                0b10 => FormatType::FormattedDisk,
                0b11 => FormatType::NoDisk,
                c => {
                    warn!("device reports capacity with format flag: {}, treating as NoDisk", c);
                    FormatType::NoDisk
                }
            };

            capacities.push(FormatCapacity { block_count, block_length, format });
        }

        Ok(capacities)
    }

    fn read_capacity(bulk: &BulkOnlyProtocol) -> USBResult<Capacity> {
        // READ CAPACITY(10)
        let mut slice_array = [0u8; 8];
        bulk.transfer(0, &[0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], TransferBuffer::Read(&mut slice_array))?;

        let last_lba32 = read_u32(slice_array.as_ref(), 0);
        if last_lba32 < 0xFFFF_FFFF {
            return Ok(Capacity {
                block_count: last_lba32 as u64, // implicitly subtracts 1 here because count is exclusive
                block_length: read_u32(slice_array.as_ref(), 4),
                lba_size: LbaSize::Size10,
            });
        }

        // READ CAPACITY(16)
        let mut slice_array = [0u8; 32];
        bulk.transfer(0, &[0x9E, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], TransferBuffer::Read(&mut slice_array))?;

        Ok(Capacity {
            block_count: read_u64(slice_array.as_ref(), 0),
            block_length: read_u32(slice_array.as_ref(), 8),
            lba_size: LbaSize::Size16,
        })
    }
}

impl SimpleBlockDevice for TransparentSCSI {
    fn sector_size(&self) -> u64 {
        self.capacity.block_length as u64
    }

    fn read_sector(&mut self, n: u64, buf: &mut [u8]) -> USBResult<usize> {
        if buf.len() % self.sector_size() as usize != 0 {
            return USBErrorKind::InvalidArgument.err("buffer len must be divisible by sector size");
        }
        let mut block_count = buf.len() / self.sector_size() as usize;

        if n + block_count as u64 > self.capacity.block_count {
            return USBErrorKind::InvalidArgument.err("cannot read past last block");
        }

        match self.capacity.lba_size {
            LbaSize::Size10 => {
                if block_count > 0xFF_FF {
                    debug!("block_count that does not fit in a READ(10) being truncated.");
                    block_count = 0xFF_FF;
                }

                let command = SCSIBuilder::new(0x28) // READ(10)
                    .push_u8(0) // reserved
                    .push_u32(n as u32)
                    .push_u8(0) // reserved
                    .push_u16(block_count as u16)
                    .push_u8(0); // control = 0

                self.bulk.transfer(0, command.as_slice(), TransferBuffer::Read(&mut buf[..block_count as usize * self.sector_size() as usize]))
            }
            LbaSize::Size16 => {
                let command = SCSIBuilder::new(0x88) // READ(16)
                    .push_u8(0) // reserved
                    .push_u64(n)
                    .push_u32(block_count as u32)
                    .push_u8(0) // reserved
                    .push_u8(0); // control = 0

                self.bulk.transfer(0, command.as_slice(), TransferBuffer::Read(&mut buf[..block_count as usize * self.sector_size() as usize]))
            }
        }
    }

    fn write_sector(&mut self, n: u64, buf: &[u8]) -> USBResult<usize> {
        if buf.len() % self.sector_size() as usize != 0 {
            return USBErrorKind::InvalidArgument.err("buffer len must be divisible by sector size");
        }
        let block_count = buf.len() / self.sector_size() as usize;


        unimplemented!()
    }
}



impl<H: UsbHAL> MassStorageDriver<H> {
    pub fn probe(device: &Arc<RwLock<USBDevice>>, interface: &USBInterfaceDescriptorSet) -> USBResult<()> {
        if interface.interface.bInterfaceSubClass != 0x6 {
            debug!("Skipping MSD with sub-class other than 0x6 (Transparent SCSI)");
            return Ok(());
        }

        if interface.interface.bInterfaceProtocol != 0x50 {
            debug!("Skipping MSD with protocol other than bulk-only");
            return Ok(());
        }

        if interface.endpoints.len() < 2 {
            return USBErrorKind::InvalidDescriptor.err("MSD has not enough endpoints!");
        }

        debug!("acquiring locks");

        let dev_lock = device.read();
        let bus_lock = dev_lock.bus.read();

        let mut input_ep: Option<Arc<RwLock<USBPipe>>> = None;
        let mut output_ep: Option<Arc<RwLock<USBPipe>>> = None;

        for endpoint in interface.endpoints.iter() {
            if !matches!(endpoint.transfer_type(), EndpointType::Bulk) {
                continue;
            }

            debug!("opening pipe: {}", endpoint.bEndpointAddress);

            let pipe = bus_lock.controller.pipe_open(device, Some(endpoint))?;

            if endpoint.is_input() {
                input_ep = Some(pipe);
            } else {
                output_ep = Some(pipe);
            }
        }

        let input_ep = input_ep.ok_or(USBErrorKind::InvalidState.msg("MSD has no bulk input endpoint"))?;
        let output_ep = output_ep.ok_or(USBErrorKind::InvalidState.msg("MSD has no bulk output endpoint"))?;

        debug!("locking endpoints");

        let bulk = BulkOnlyProtocol {  input_pipe: input_ep, output_pipe: output_ep };

        let mut scsi = TransparentSCSI::new(bulk)?;

        let mut buf: Vec<u8> = Vec::new();
        buf.resize(512 * 1, 0);

        scsi.read_sector(0, buf.as_mut_slice())?;

        info!("Fist 16 sectors:\n{}", pretty_hex::pretty_hex(&buf.as_slice()));

        H::sleep(Duration::from_secs(5));
        Ok(())
    }
}

