use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;

use spin::RwLock;

use crate::structs::USBPipe;
use crate::USBResult;

pub type PipeAsyncListener = Arc<dyn Fn(&[u8], USBResult<usize>) + Send + Sync>;

fn do_pipe_async(pipe: Arc<RwLock<USBPipe>>, buf: Vec<u8>, func: PipeAsyncListener) -> USBResult<()> {
    let pipe_cloned = pipe.clone();
    let pipe_lock = pipe.read();
    pipe_lock.async_read(buf, Box::new(move |buf, result| {
        func(buf.as_slice(), result);
        if let Err(e) = do_pipe_async(pipe_cloned, buf, func) {
            warn!("failed to register pipe async: {:?}", e);
        }
    }))
}

pub fn pipe_async_listener(pipe: Arc<RwLock<USBPipe>>, num_entries: usize, buf_size: usize, func: PipeAsyncListener) -> USBResult<()> {
    for _ in 0..num_entries {
        let mut buf = Vec::new();
        buf.reserve_exact(if buf_size < 128 { 128 } else { buf_size });
        buf.resize(buf_size, 0);
        do_pipe_async(pipe.clone(), buf, func.clone())?;
    }
    Ok(())
}




