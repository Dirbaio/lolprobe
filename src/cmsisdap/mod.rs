//! CMSIS-DAP probe implementation.
pub mod commands;
pub mod tools;

use std::fmt::{self, Write};

use anyhow::anyhow;
use bitvec::prelude::*;
use commands::general::connect::{ConnectRequest, ConnectResponse};
use commands::general::disconnect::{DisconnectRequest, DisconnectResponse};
use commands::general::host_status::{HostStatusRequest, HostStatusResponse};
use commands::general::info::{Capabilities, CapabilitiesCommand, PacketCountCommand, SWOTraceBufferSizeCommand};
use commands::general::reset::{ResetRequest, ResetResponse};
use commands::jtag::configure::ConfigureRequest as JtagConfigureRequest;
use commands::jtag::sequence::{
    Sequence as JtagSequence, SequenceRequest as JtagSequenceRequest, SequenceResponse as JtagSequenceResponse,
};
use commands::swj::clock::SWJClockRequest;
use commands::swj::pins::{SWJPinsRequestBuilder, SWJPinsResponse};
use commands::swj::sequence::{SequenceRequest, SequenceResponse};
use commands::transfer::configure::ConfigureRequest;
use commands::transfer::{Ack, Port, TransferRequest};
use commands::{swd, swo, CmsisDapDevice, CmsisDapError, RequestError, Status};

/// An error in the communication with an access port or
/// debug port.
#[derive(Debug, thiserror::Error, Clone, PartialEq, Eq, Copy)]
pub enum DapError {
    /// An error occurred during SWD communication.
    #[error("An error occurred in the SWD communication between probe and device.")]
    SwdProtocol,
    /// The target device did not respond to the request.
    #[error("Target device did not respond to request.")]
    NoAcknowledge,
    /// The target device responded with a FAULT response to the request.
    #[error("Target device responded with a FAULT response to the request.")]
    FaultResponse,
    /// Target device responded with a WAIT response to the request.
    #[error("Target device responded with a WAIT response to the request.")]
    WaitResponse,
    /// The parity bit on the read request was incorrect.
    #[error("Incorrect parity on READ request.")]
    IncorrectParity,
}

/// The protocol that is to be used by the probe when communicating with the target.
///
/// For ARM select `Swd` or `Jtag`, for RISC-V select `Jtag`.
#[derive(Copy, Clone, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub enum WireProtocol {
    /// Serial Wire Debug is ARMs proprietary standard for communicating with ARM cores.
    /// You can find specifics in the [`ARM Debug Interface v5.2`](https://developer.arm.com/documentation/ihi0031/f/?lang=en) specification.
    Swd,
    /// JTAG is a standard which is supported by many chips independent of architecture.
    /// See [`Wikipedia`](https://en.wikipedia.org/wiki/JTAG) for more info.
    Jtag,
}

impl fmt::Display for WireProtocol {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            WireProtocol::Swd => f.write_str("SWD"),
            WireProtocol::Jtag => f.write_str("JTAG"),
        }
    }
}

impl std::str::FromStr for WireProtocol {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match &s.to_ascii_lowercase()[..] {
            "swd" => Ok(WireProtocol::Swd),
            "jtag" => Ok(WireProtocol::Jtag),
            _ => Err(format!("'{s}' is not a valid protocol. Choose from [swd, jtag].")),
        }
    }
}

/// A command queued in a batch for later execution
///
/// Mostly used internally but returned in anyhow::Error to indicate
/// which batched command actually encountered the error.
#[derive(Copy, Clone, Debug)]
pub enum BatchCommand {
    /// Read from a port
    Read(Port, u16),

    /// Write to a port
    Write(Port, u16, u32),
}

impl fmt::Display for BatchCommand {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            BatchCommand::Read(port, addr) => {
                write!(f, "Read(port={port:?}, addr={addr})")
            }
            BatchCommand::Write(port, addr, data) => {
                write!(f, "Write(port={port:?}, addr={addr}, data={data:#010x})")
            }
        }
    }
}

/// A CMSIS-DAP probe.
pub struct CmsisDap {
    device: CmsisDapDevice,
    _hw_version: u8,
    _jtag_version: u8,
    protocol: Option<WireProtocol>,

    packet_size: u16,
    packet_count: u8,
    capabilities: Capabilities,
    swo_buffer_size: Option<usize>,
    swo_active: bool,
    swo_streaming: bool,
    connected: bool,

    /// Speed in kHz
    speed_khz: u32,

    batch: Vec<BatchCommand>,
}

impl std::fmt::Debug for CmsisDap {
    fn fmt(&self, fmt: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        fmt.debug_struct("CmsisDap")
            .field("protocol", &self.protocol)
            .field("packet_size", &self.packet_size)
            .field("packet_count", &self.packet_count)
            .field("capabilities", &self.capabilities)
            .field("swo_buffer_size", &self.swo_buffer_size)
            .field("swo_active", &self.swo_active)
            .field("swo_streaming", &self.swo_streaming)
            .field("speed_khz", &self.speed_khz)
            .finish()
    }
}

impl CmsisDap {
    pub fn new_from_device(mut device: CmsisDapDevice) -> Result<Self, anyhow::Error> {
        // Discard anything left in buffer, as otherwise
        // we'll get out of sync between requests and responses.
        device.drain();

        // Determine and set the packet size. We do this as soon as possible after
        // opening the probe to ensure all future communication uses the correct size.
        let packet_size = device.find_packet_size()? as u16;

        // Read remaining probe information.
        let packet_count = commands::send_command(&mut device, &PacketCountCommand {})?;
        let caps: Capabilities = commands::send_command(&mut device, &CapabilitiesCommand {})?;
        tracing::debug!("Detected probe capabilities: {:?}", caps);
        let mut swo_buffer_size = None;
        if caps.swo_uart_implemented || caps.swo_manchester_implemented {
            let swo_size = commands::send_command(&mut device, &SWOTraceBufferSizeCommand {})?;
            swo_buffer_size = Some(swo_size as usize);
            tracing::debug!("Probe SWO buffer size: {}", swo_size);
        }

        Ok(Self {
            device,
            _hw_version: 0,
            _jtag_version: 0,
            protocol: None,
            packet_count,
            packet_size,
            capabilities: caps,
            swo_buffer_size,
            swo_active: false,
            swo_streaming: false,
            connected: false,
            speed_khz: 1_000,
            batch: Vec::new(),
        })
    }

    /// Set maximum JTAG/SWD clock frequency to use, in Hz.
    ///
    /// The actual clock frequency used by the device might be lower.
    fn set_swj_clock(&mut self, clock_speed_hz: u32) -> Result<(), CmsisDapError> {
        let request = SWJClockRequest { clock_speed_hz };
        commands::send_command(&mut self.device, &request)
            .map_err(CmsisDapError::from)
            .and_then(|v| match v.status {
                Status::DapOk => Ok(()),
                Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::SWJClock { request })),
            })
    }

    fn transfer_configure(&mut self, request: ConfigureRequest) -> Result<(), CmsisDapError> {
        commands::send_command(&mut self.device, &request)
            .map_err(CmsisDapError::from)
            .and_then(|v| match v.status {
                Status::DapOk => Ok(()),
                Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::TransferConfigure {
                    request,
                })),
            })
    }

    fn configure_swd(&mut self, request: swd::configure::ConfigureRequest) -> Result<(), CmsisDapError> {
        commands::send_command(&mut self.device, &request)
            .map_err(CmsisDapError::from)
            .and_then(|v| match v.status {
                Status::DapOk => Ok(()),
                Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::SwdConfigure { request })),
            })
    }

    /// Reset JTAG state machine to Test-Logic-Reset.
    fn jtag_ensure_test_logic_reset(&mut self) -> Result<(), CmsisDapError> {
        let sequence = JtagSequence::no_capture(true, &bitvec![u8, Lsb0; 0; 6])?;
        let sequences = vec![sequence];

        self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?;

        Ok(())
    }

    /// Reset JTAG state machine to Run-Test/Idle, as requisite precondition for DAP_Transfer commands.
    fn jtag_ensure_run_test_idle(&mut self) -> Result<(), CmsisDapError> {
        // These could be coalesced into one sequence request, but for now we'll keep things simple.

        // First reach Test-Logic-Reset
        self.jtag_ensure_test_logic_reset()?;

        // Then transition to Run-Test-Idle
        let sequence = JtagSequence::no_capture(false, &bitvec![u8, Lsb0; 0; 1])?;
        let sequences = vec![sequence];
        self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?;

        Ok(())
    }

    /// Capture the power-up scan chain values, including all IDCODEs.
    ///
    /// Returns the IR and DR results as (IR, DR).
    fn jtag_reset_scan(&mut self) -> Result<(BitVec<u8>, BitVec<u8>), CmsisDapError> {
        let dr = self.jtag_scan_dr()?;
        let ir = self.jtag_scan_ir()?;

        // Return to Run-Test/Idle, so the probe is ready for DAP_Transfer commands again.
        self.jtag_ensure_run_test_idle()?;

        Ok((ir, dr))
    }

    /// Detect the IR chain length and return its current contents.
    ///
    /// Replaces the current contents with all 1s (BYPASS) and enters
    /// the Run-Test/Idle state.
    fn jtag_scan_ir(&mut self) -> Result<BitVec<u8>, CmsisDapError> {
        self.jtag_ensure_shift_ir()?;
        let data = self.jtag_scan_inner("IR")?;
        Ok(data)
    }

    /// Detect the DR chain length and return its contents.
    ///
    /// Replaces the current contents with all 1s and enters
    /// the Run-Test/Idle state.
    fn jtag_scan_dr(&mut self) -> Result<BitVec<u8>, CmsisDapError> {
        self.jtag_ensure_shift_dr()?;
        let data = self.jtag_scan_inner("DR")?;
        Ok(data)
    }

    /// Detect current chain length and return its contents.
    /// Must already be in either Shift-IR or Shift-DR state.
    fn jtag_scan_inner(&mut self, name: &'static str) -> Result<BitVec<u8>, CmsisDapError> {
        // Max scan chain length (in bits) to attempt to detect.
        const MAX_LENGTH: usize = 128;
        // How many bytes to write out / read in per request.
        const BYTES_PER_REQUEST: usize = 16;
        // How many requests are needed to read/write at least MAX_LENGTH bits.
        const REQUESTS: usize = MAX_LENGTH.div_ceil(BYTES_PER_REQUEST * 8);

        // Completely fill xR with 0s, capture result.
        let mut tdo_bytes: Vec<u8> = Vec::with_capacity(REQUESTS * BYTES_PER_REQUEST);
        for _ in 0..REQUESTS {
            let sequences = vec![
                JtagSequence::capture(false, &bitvec![u8, Lsb0; 0; 64])?,
                JtagSequence::capture(false, &bitvec![u8, Lsb0; 0; 64])?,
            ];

            tdo_bytes.extend(self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?.iter());
        }
        let d0 = tdo_bytes.view_bits::<Lsb0>();

        // Completely fill xR with 1s, capture result.
        let mut tdo_bytes: Vec<u8> = Vec::with_capacity(REQUESTS * BYTES_PER_REQUEST);
        for _ in 0..REQUESTS {
            let sequences = vec![
                JtagSequence::capture(false, &bitvec![u8, Lsb0; 1; 64])?,
                JtagSequence::capture(false, &bitvec![u8, Lsb0; 1; 64])?,
            ];

            tdo_bytes.extend(self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?.iter());
        }
        let d1 = tdo_bytes.view_bits::<Lsb0>();

        // Find first 1 in d1, which indicates length of register.
        let n = match d1.first_one() {
            Some(n) => {
                tracing::info!("JTAG {name} scan chain detected as {n} bits long");
                n
            }
            None => {
                let expected_bit = 1;
                tracing::error!("JTAG {name} scan chain either broken or too long: did not detect {expected_bit}");
                return Err(CmsisDapError::ErrorResponse(RequestError::BrokenScanChain {
                    name,
                    expected_bit,
                }));
            }
        };

        // Check at least one register is detected in the scan chain.
        if n == 0 {
            tracing::error!("JTAG {name} scan chain is empty");
            return Err(CmsisDapError::ErrorResponse(RequestError::EmptyScanChain { name }));
        }

        // Check d0[n..] are all 0.
        if d0[n..].any() {
            let expected_bit = 0;
            tracing::error!("JTAG {name} scan chain either broken or too long: did not detect {expected_bit}");
            return Err(CmsisDapError::ErrorResponse(RequestError::BrokenScanChain {
                name,
                expected_bit,
            }));
        }

        // Extract d0[..n] as the initial scan chain contents.
        let data = d0[..n].to_bitvec();

        Ok(data)
    }

    fn jtag_ensure_shift_dr(&mut self) -> Result<(), CmsisDapError> {
        // Transition to Test-Logic-Reset.
        self.jtag_ensure_test_logic_reset()?;

        // Transition to Shift-DR
        let sequences = vec![
            JtagSequence::no_capture(false, &bitvec![u8, Lsb0; 0; 1])?,
            JtagSequence::no_capture(true, &bitvec![u8, Lsb0; 0; 1])?,
            JtagSequence::no_capture(false, &bitvec![u8, Lsb0; 0; 2])?,
        ];
        self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?;

        Ok(())
    }

    fn jtag_ensure_shift_ir(&mut self) -> Result<(), CmsisDapError> {
        // Transition to Test-Logic-Reset.
        self.jtag_ensure_test_logic_reset()?;

        // Transition to Shift-IR
        let sequences = vec![
            JtagSequence::no_capture(false, &bitvec![u8, Lsb0; 0; 1])?,
            JtagSequence::no_capture(true, &bitvec![u8, Lsb0; 0; 2])?,
            JtagSequence::no_capture(false, &bitvec![u8, Lsb0; 0; 2])?,
        ];
        self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?;

        Ok(())
    }

    fn send_jtag_configure(&mut self, request: JtagConfigureRequest) -> Result<(), CmsisDapError> {
        commands::send_command(&mut self.device, &request)
            .map_err(CmsisDapError::from)
            .and_then(|v| match v.status {
                Status::DapOk => Ok(()),
                Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::JtagConfigure { request })),
            })
    }

    fn send_jtag_sequences(&mut self, request: JtagSequenceRequest) -> Result<Vec<u8>, CmsisDapError> {
        commands::send_command(&mut self.device, &request)
            .map_err(CmsisDapError::from)
            .and_then(|v| match v {
                JtagSequenceResponse(Status::DapOk, tdo) => Ok(tdo),
                JtagSequenceResponse(Status::DapError, _) => {
                    Err(CmsisDapError::ErrorResponse(RequestError::JtagSequence { request }))
                }
            })
    }

    pub fn send_swj_sequences(&mut self, request: SequenceRequest) -> Result<(), CmsisDapError> {
        // Ensure all pending commands are processed.
        //self.process_batch()?;

        commands::send_command(&mut self.device, &request)
            .map_err(CmsisDapError::from)
            .and_then(|v| match v {
                SequenceResponse(Status::DapOk) => Ok(()),
                SequenceResponse(Status::DapError) => {
                    Err(CmsisDapError::ErrorResponse(RequestError::SwjSequence { request }))
                }
            })
    }

    /// Set SWO port to use requested transport.
    ///
    /// Check the probe capabilities to determine which transports are available.
    fn set_swo_transport(&mut self, transport: swo::TransportRequest) -> Result<(), anyhow::Error> {
        let response = commands::send_command(&mut self.device, &transport)?;
        match response.status {
            Status::DapOk => Ok(()),
            Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::SwoTransport { transport }).into()),
        }
    }

    /// Set SWO port to specified mode.
    ///
    /// Check the probe capabilities to determine which modes are available.
    fn set_swo_mode(&mut self, mode: swo::ModeRequest) -> Result<(), anyhow::Error> {
        let response = commands::send_command(&mut self.device, &mode)?;
        match response.status {
            Status::DapOk => Ok(()),
            Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::SwoMode { mode }).into()),
        }
    }

    /// Set SWO port to specified baud rate.
    ///
    /// Returns `SwoBaudrateNotConfigured` if the probe returns 0,
    /// indicating the requested baud rate was not configured,
    /// and returns the configured baud rate on success (which
    /// may differ from the requested baud rate).
    fn set_swo_baudrate(&mut self, request: swo::BaudrateRequest) -> Result<u32, anyhow::Error> {
        let response = commands::send_command(&mut self.device, &request)?;
        tracing::debug!("Requested baud {}, got {}", request.baudrate, response);
        if response == 0 {
            Err(CmsisDapError::SwoBaudrateNotConfigured.into())
        } else {
            Ok(response)
        }
    }

    /// Start SWO trace data capture.
    fn start_swo_capture(&mut self) -> Result<(), anyhow::Error> {
        let command = swo::ControlRequest::Start;
        let response = commands::send_command(&mut self.device, &command)?;
        match response.status {
            Status::DapOk => Ok(()),
            Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::SwoControl { command }).into()),
        }
    }

    /// Stop SWO trace data capture.
    fn stop_swo_capture(&mut self) -> Result<(), anyhow::Error> {
        let command = swo::ControlRequest::Stop;
        let response = commands::send_command(&mut self.device, &command)?;
        match response.status {
            Status::DapOk => Ok(()),
            Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::SwoControl { command }).into()),
        }
    }

    /// Fetch current SWO trace status.
    #[allow(dead_code)]
    fn get_swo_status(&mut self) -> Result<swo::StatusResponse, anyhow::Error> {
        Ok(commands::send_command(&mut self.device, &swo::StatusRequest)?)
    }

    /// Fetch extended SWO trace status.
    ///
    /// request.request_status: request trace status
    /// request.request_count: request remaining bytes in trace buffer
    /// request.request_index: request sequence number and timestamp of next trace sequence
    #[allow(dead_code)]
    fn get_swo_extended_status(
        &mut self,
        request: swo::ExtendedStatusRequest,
    ) -> Result<swo::ExtendedStatusResponse, anyhow::Error> {
        Ok(commands::send_command(&mut self.device, &request)?)
    }

    /// Fetch latest SWO trace data by sending a DAP_SWO_Data request.
    fn get_swo_data(&mut self) -> Result<Vec<u8>, anyhow::Error> {
        match self.swo_buffer_size {
            Some(swo_buffer_size) => {
                // We'll request the smaller of the probe's SWO buffer and
                // its maximum packet size. If the probe has less data to
                // send it will respond with as much as it can.
                let n = usize::min(swo_buffer_size, self.packet_size as usize) as u16;

                let response: swo::DataResponse =
                    commands::send_command(&mut self.device, &swo::DataRequest { max_count: n })?;
                if response.status.error {
                    Err(CmsisDapError::SwoTraceStreamError.into())
                } else {
                    Ok(response.data)
                }
            }
            None => Ok(Vec::new()),
        }
    }

    fn connect_if_needed(&mut self) -> Result<(), anyhow::Error> {
        if self.connected {
            return Ok(());
        }

        let protocol: ConnectRequest = if let Some(protocol) = self.protocol {
            match protocol {
                WireProtocol::Swd => ConnectRequest::Swd,
                WireProtocol::Jtag => ConnectRequest::Jtag,
            }
        } else {
            ConnectRequest::DefaultPort
        };

        let used_protocol = commands::send_command(&mut self.device, &protocol)
            .map_err(CmsisDapError::from)
            .and_then(|v| match v {
                ConnectResponse::SuccessfulInitForSWD => Ok(WireProtocol::Swd),
                ConnectResponse::SuccessfulInitForJTAG => Ok(WireProtocol::Jtag),
                ConnectResponse::InitFailed => Err(CmsisDapError::ErrorResponse(RequestError::InitFailed {
                    protocol: self.protocol,
                })),
            })?;

        // Store the actually used protocol, to handle cases where the default protocol is used.
        tracing::info!("Using protocol {}", used_protocol);
        self.protocol = Some(used_protocol);
        self.connected = true;

        Ok(())
    }

    fn get_name(&self) -> &str {
        "CMSIS-DAP"
    }

    /// Get the currently set maximum speed.
    ///
    /// CMSIS-DAP offers no possibility to get the actual speed used.
    fn speed_khz(&self) -> u32 {
        self.speed_khz
    }

    /// For CMSIS-DAP, we can set the maximum speed. The actual speed
    /// used by the probe cannot be determined, but it will not be
    /// higher than this value.
    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, anyhow::Error> {
        self.set_swj_clock(speed_khz * 1_000)?;
        self.speed_khz = speed_khz;

        Ok(speed_khz)
    }

    /// Enters debug mode.
    #[tracing::instrument(skip(self))]
    fn attach(&mut self) -> Result<(), anyhow::Error> {
        tracing::debug!("Attaching to target system (clock = {}kHz)", self.speed_khz);

        // Run connect sequence (may already be done earlier via swj operations)
        self.connect_if_needed()?;

        // Set speed after connecting as it can be reset during protocol selection
        self.set_speed(self.speed_khz)?;

        self.transfer_configure(ConfigureRequest {
            idle_cycles: 0,
            wait_retry: 0xffff,
            match_retry: 0,
        })?;

        if self.active_protocol() == Some(WireProtocol::Jtag) {
            // no-op: we configure JTAG in debug_port_setup,
            // because that is where we execute the SWJ-DP Switch Sequence
            // to ensure the debug port is ready for JTAG signals,
            // at which point we can interrogate the scan chain
            // and configure the probe with the given IR lengths.
        } else {
            self.configure_swd(swd::configure::ConfigureRequest {})?;
        }

        // Tell the probe we are connected so it can turn on an LED.
        let _: Result<HostStatusResponse, _> =
            commands::send_command(&mut self.device, &HostStatusRequest::connected(true));

        Ok(())
    }

    /// Leave debug mode.
    fn detach(&mut self) -> Result<(), anyhow::Error> {
        //self.process_batch()?;
        //
        //if self.swo_active {
        //    self.disable_swo()?;
        //}

        let response = commands::send_command(&mut self.device, &DisconnectRequest {}).map_err(anyhow::Error::from)?;

        // Tell probe we are disconnected so it can turn off its LED.
        let request = HostStatusRequest::connected(false);
        let _: Result<HostStatusResponse, _> = commands::send_command(&mut self.device, &request);

        self.connected = false;

        match response {
            DisconnectResponse(Status::DapOk) => Ok(()),
            DisconnectResponse(Status::DapError) => {
                Err(CmsisDapError::ErrorResponse(RequestError::HostStatus { request }).into())
            }
        }
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), anyhow::Error> {
        match protocol {
            WireProtocol::Jtag if self.capabilities._jtag_implemented => {
                self.protocol = Some(WireProtocol::Jtag);
                Ok(())
            }
            WireProtocol::Swd if self.capabilities._swd_implemented => {
                self.protocol = Some(WireProtocol::Swd);
                Ok(())
            }
            _ => Err(anyhow!("unsupported protocol {protocol}")),
        }
    }

    fn active_protocol(&self) -> Option<WireProtocol> {
        self.protocol
    }

    /// Asserts the nRESET pin.
    fn target_reset(&mut self) -> Result<(), anyhow::Error> {
        commands::send_command(&mut self.device, &ResetRequest).map(|v: ResetResponse| {
            tracing::info!("Target reset response: {:?}", v);
        })?;
        Ok(())
    }

    fn target_reset_assert(&mut self) -> Result<(), anyhow::Error> {
        let request = SWJPinsRequestBuilder::new().nreset(false).build();

        commands::send_command(&mut self.device, &request).map(|v: SWJPinsResponse| {
            tracing::info!("Pin response: {:?}", v);
        })?;
        Ok(())
    }

    fn target_reset_deassert(&mut self) -> Result<(), anyhow::Error> {
        let request = SWJPinsRequestBuilder::new().nreset(true).build();

        commands::send_command(&mut self.device, &request).map(|v: SWJPinsResponse| {
            tracing::info!("Pin response: {:?}", v);
        })?;
        Ok(())
    }

    pub fn read(&mut self, port: Port, register: u8) -> Result<u32, anyhow::Error> {
        let mut transfers = TransferRequest::empty();
        transfers.add_read(port, register);
        let response = commands::send_command(&mut self.device, &transfers)?;
        assert_eq!(response.last_transfer_response.ack, Ack::Ok);
        assert_eq!(response.last_transfer_response.protocol_error, false);
        Ok(response.transfers[0].data.unwrap())
    }

    pub fn write(&mut self, port: Port, register: u8, value: u32) -> Result<(), anyhow::Error> {
        let mut transfers = TransferRequest::empty();
        transfers.add_write(port, register, value);
        let response = commands::send_command(&mut self.device, &transfers)?;
        assert_eq!(response.last_transfer_response.ack, Ack::Ok);
        assert_eq!(response.last_transfer_response.protocol_error, false);
        Ok(())
    }

    pub fn swj_sequence(&mut self, bit_len: u8, bits: u64) -> Result<(), anyhow::Error> {
        self.connect_if_needed()?;

        let data = bits.to_le_bytes();

        if tracing::enabled!(tracing::Level::TRACE) {
            let mut seq = String::new();

            let _ = write!(&mut seq, "swj sequence:");

            for i in 0..bit_len {
                let bit = (bits >> i) & 1;

                if bit == 1 {
                    let _ = write!(&mut seq, "1");
                } else {
                    let _ = write!(&mut seq, "0");
                }
            }
            tracing::trace!("{}", seq);
        }

        self.send_swj_sequences(SequenceRequest::new(&data, bit_len)?)?;

        Ok(())
    }
}
