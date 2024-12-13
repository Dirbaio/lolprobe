#![recursion_limit = "256"] // required by bitfield!
#![deny(unused_must_use)]

use std::rc::Rc;

use ap::Ap;
use cmsisdap::commands::send_command;
use cmsisdap::commands::transfer::Port;
use cmsisdap::tools::{list_cmsisdap_devices, open_device_from_selector, open_v2_device, DebugProbeSelector};
use cmsisdap::CmsisDap;
use dp::{Dp, DpAddress};
use probe::Probe;

mod ap;
mod cmsisdap;
mod dp;
mod probe;

fn main() -> Result<(), anyhow::Error> {
    tracing_subscriber::fmt::init();

    let devs = list_cmsisdap_devices();
    let dev = &devs[0];
    let dev = open_device_from_selector(&DebugProbeSelector {
        product_id: dev.product_id,
        vendor_id: dev.vendor_id,
        serial_number: dev.serial_number.clone(),
    })?;
    let dev = CmsisDap::new_from_device(dev)?;
    let probe = Rc::new(Probe::new(dev));

    let dp_core0 = Rc::new(Dp::new(probe.clone(), DpAddress::Multidrop(0x01002927)));
    let dp_core1 = Rc::new(Dp::new(probe.clone(), DpAddress::Multidrop(0x11002927)));

    let x = dp_core0.read(0)?;
    tracing::debug!("dpidr {:#x}", x);

    //let x = dp_core1.read(0)?;
    //tracing::debug!("dpidr {:#x}", x);

    dp_core0.powerup()?;
    dp_core1.powerup()?;

    let ap = Ap::new(dp_core0.clone(), 0);
    let x = ap.read(0xFC)?;
    tracing::debug!("ap id {:#x}", x);
    let x = ap.read(0xFC)?;
    tracing::debug!("ap id {:#x}", x);

    let inner = &mut *probe.inner.borrow_mut();
    let x = inner.dev.read(Port::Ap, 0xC)?;
    tracing::debug!("wololo {:#x}", x);
    let x = inner.dev.read(Port::Ap, 0x8)?;
    tracing::debug!("wololo {:#x}", x);
    let x = inner.dev.read(Port::Dp, 0xC)?;
    tracing::debug!("wololo {:#x}", x);

    Ok(())
}
