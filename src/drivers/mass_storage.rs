use alloc::sync::Arc;
use alloc::vec::Vec;
use core::marker::PhantomData;
use core::time::Duration;

use spin::RwLock;

use crate::{HAL2, USBErrorKind, USBResult, as_slice, as_mut_slice};
use crate::descriptor::USBInterfaceDescriptorSet;
use crate::structs::{USBDevice, USBPipe};
use crate::items::{EndpointType, TransferBuffer};

pub struct MassStorageDriver<H: HAL2> {
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
    pub fn transfer(&self, lun: u8, command: &[u8], buffer: TransferBuffer) -> USBResult<usize> {
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


impl<H: HAL2> MassStorageDriver<H> {
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

        let mut slice = [0u8; 36];
        bulk.transfer(0, &[0x12, 0x00, 0x00, 0x00, 0x24, 0x00], TransferBuffer::Read(&mut slice))?;

        info!("Inquiry completed:\n{}", pretty_hex::pretty_hex(&(slice.as_ref())));

        let mut slice = [0u8; 252];
        bulk.transfer(0, &[0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00], TransferBuffer::Read(&mut slice))?;

        info!("READ FORMAT CAPACITIES completed:\n{}", pretty_hex::pretty_hex(&(slice.as_ref())));

        H::sleep(Duration::from_secs(5));
        Ok(())
    }
}

