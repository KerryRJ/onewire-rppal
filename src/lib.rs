use std::fmt::{Debug, Display, Formatter};
use std::time::{Duration, Instant};
use rppal::gpio::{IoPin, Mode};

const ADDRESS_BITS: u8 = u64::BITS as u8;

#[derive(Debug, Copy, Clone)]
pub enum OnewireError {
    CrcMismatch,
    DevicesNotDetected,
    InputPinError,
    LevelIsNotHigh,
    ShortDetected,
    UnexpectedResponse,
}

pub type OnewireResult<T> = Result<T, OnewireError>;

#[repr(u8)]
pub enum Commands {
    Read = 0x33,
    Match = 0x55,
    Skip = 0xCC,
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum SearchCommands {
    Normal = 0xF0,
    Conditional = 0xEC,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Searching {
    Initialized,
    Busy,
    Completed,
}

impl Default for Searching {
    fn default() -> Self {
        Searching::Initialized
    }
}

#[derive(Clone)]
pub struct SearchAlgorithmState {
    registration: u64,
    last_discrepancy: u8,
    searching: Searching,
}

impl SearchAlgorithmState {
    #[inline]
    fn is_bit_set(data: &u64, bit: u8) -> bool {
        if bit > ADDRESS_BITS {
            return false;
        }
        data & (1 << bit) != 0    
    }

    #[inline]
    fn clear_bit(data: &mut u64, bit: u8) {
        if bit > ADDRESS_BITS {
            return;
        }
        *data &= !(1 << bit)
    }

    #[inline]
    fn set_bit(data: &mut u64, bit: u8) {
        if bit > ADDRESS_BITS {
            return;
        }
        *data |= 1 << bit
    }

    #[inline]
    fn write_bit(data: &mut u64, bit: u8, value: bool) {
        if value {
            SearchAlgorithmState::set_bit(data, bit);
        } else {
            SearchAlgorithmState::clear_bit(data, bit);
        }
    }
}

impl Default for SearchAlgorithmState {
    fn default() -> Self {
        SearchAlgorithmState {
            registration: 0,
            last_discrepancy: 0,
            searching: Searching::default(),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Registration {
    family: u8,
    serial_number: [u8; 6],
    crc: u8,
}

impl From<u64> for Registration {
    fn from(rom: u64) -> Self {
        let bytes= rom.to_le_bytes();
        Registration {
            family: bytes[0],
            serial_number: [bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6]],
            crc: bytes[7],
        }
    }
}

impl From<Registration> for u64 {
    fn from(registration: Registration) -> Self {
        u64::from_be_bytes(
            [
                registration.crc,
                registration.serial_number[5],
                registration.serial_number[4],
                registration.serial_number[3],
                registration.serial_number[2],
                registration.serial_number[1],
                registration.serial_number[0],
                registration.family,
            ]
        )
    }
}

impl Display for Registration {
    fn fmt(&self, f: &mut Formatter) -> core::fmt::Result {
        write!(
            f,
            "Family: {:02X}, S/N: {:02X}{:02X}{:02X}{:02X}{:02X}{:02X}, CRC: {:02X}",
            self.family,
            self.serial_number[5], self.serial_number[4], self.serial_number[3], self.serial_number[2], self.serial_number[1], self.serial_number[0],
            self.crc,
        )
    }
}

const LOOKUP_TABLE: [u8; 32] = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74,
];

fn calculate_crc8(data: &[u8]) -> u8 {
    let mut crc = 0;
    for byte in data {
        crc = *byte ^ crc;
        crc = LOOKUP_TABLE[(crc & 0x0f) as usize] ^ LOOKUP_TABLE[(crc >> 4 & 0x0f) as usize + 16];
    }
    crc
}

pub struct Onewire {
    pin: IoPin,
}

impl Onewire {
    pub fn new(pin: IoPin) -> Self {
        Onewire {
            pin: pin,
        }
    }

    #[inline(always)]
    fn sleep(&mut self, duration: Duration) {
        let start = Instant::now();
        while start.elapsed() < duration {
        }
    }

    #[inline(always)]
    fn ensure_level_is_high(&mut self) -> OnewireResult<()> {
        for _ in 0..125 {
            if self.pin.is_high() {
                return Ok(());
            }
            self.sleep(Duration::from_micros(2));
        }
        Err(OnewireError::LevelIsNotHigh)
    }

    #[inline(always)]
    pub fn reset(&mut self) -> OnewireResult<()> {
        self.ensure_level_is_high().unwrap();
        self.pin.set_mode(Mode::Output);
        self.pin.set_low();
        self.sleep(Duration::from_micros(480));
        self.pin.set_mode(Mode::Input);
        self.sleep(Duration::from_micros(70));
        let presence_detected = self.pin.is_low();
        self.sleep(Duration::from_micros(410));
        match presence_detected {
            true => Ok(()),
            false => Err(OnewireError::DevicesNotDetected),
        }
    }

    #[inline(always)]
    pub fn read_bit(&mut self) -> bool {
        self.pin.set_mode(Mode::Output);
        self.pin.set_low();
        self.pin.set_mode(Mode::Input);
        self.sleep(Duration::from_micros(5));
        let value = self.pin.is_high();
        self.sleep(Duration::from_micros(45));
        value
    }

    #[inline(always)]
    pub fn write_bit(&mut self, bit: bool) {
        self.pin.set_mode(Mode::Output);
        self.pin.set_low();
        self.sleep(if bit {Duration::from_micros(5)} else {Duration::from_micros(60)});
        self.pin.set_mode(Mode::Input);
        self.sleep(if bit {Duration::from_micros(55)} else {Duration::from_micros(5)});
    }

    #[inline(always)]
    pub fn write_byte(&mut self, mut byte: u8) {
        for _ in 0..8 {
            self.write_bit((byte & 0x1) == 0x1);
            byte >>= 1;
        }
    }
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.write_byte(*byte);
        }
    }

    #[inline(always)]
    pub fn read_byte(&mut self) -> u8 {
        let mut byte = 0;
        for _ in 0..8 {
            byte >>= 1;
            if self.read_bit() {
                byte |= 0x80;
            }
        }
        byte
    }
    pub fn read_bytes(&mut self, bytes: &mut [u8]) {
        for i in 0..bytes.len() {
            bytes[i] = self.read_byte();
        }
    }

    fn search(&mut self, command: SearchCommands, state: &mut SearchAlgorithmState) -> OnewireResult<Option<Registration>> {
        if state.searching == Searching::Completed {
            return Ok(None);
        }

        self.reset().unwrap();
        self.write_byte(command as u8);

        let mut z = 0;
        let registration = state.registration;
        for i in 0..ADDRESS_BITS {
            let bit = self.read_bit();
            let complement = self.read_bit();

            if bit && complement {
                state.registration = registration;
                return Ok(None);
            }

            let direction =
                if bit || complement {
                    // devices have a 0 or 1
                    bit
                } else {
                    // discrepancy
                    if i < state.last_discrepancy {
                        SearchAlgorithmState::is_bit_set(&mut state.registration, i)                                
                    } else if i == state.last_discrepancy {
                        !SearchAlgorithmState::is_bit_set(&mut state.registration, i)
                    } else {
                        z = i;
                        false
                    }
                };
            SearchAlgorithmState::write_bit(&mut state.registration, i, direction);
            self.write_bit(direction);
        }

        state.last_discrepancy = z;
        if state.last_discrepancy == 0 {
            state.searching = Searching::Completed;
        } else {
            state.searching = Searching::Busy;
        }

        if calculate_crc8(&state.registration.to_le_bytes()) == 0 {
            Ok(Some(Registration::from(state.registration)))
        } else {
            Err(OnewireError::CrcMismatch)
        }
    }

    /// Determine whether a device is available on the bus
    pub fn is_device_connected(&mut self, registration: Registration) -> bool {
        let mut state =
            SearchAlgorithmState {
                registration: registration.clone().into(),
                last_discrepancy: ADDRESS_BITS - 1,
                searching: Searching::default(),
            };

        match self.search(SearchCommands::Normal, &mut state) {
            Ok(o) => {
                match o {
                    Some(r) => r == registration,
                    None => false
                }
            }
            Err(_) => false
        }
    }

    /// Returns a registration iterator that can be filtered to devices that have had an alarm triggered
    pub fn registrations<'a>(&'a mut self, command: SearchCommands) -> OnewireIterator<'a> {
        OnewireIterator {
            wire: self,
            command: command,
            state: SearchAlgorithmState::default(),
        }
    }

    /// Read a registration code on a single-drop bus.
    pub fn read_registration(&mut self) -> OnewireResult<Registration> {
        self.reset().unwrap();
        self.write_byte(Commands::Read as u8);
        let mut bytes = [0u8; 8];
        self.read_bytes(&mut bytes);
        Ok(Registration::from(u64::from_le_bytes(bytes)))
    }

    /// Address a specific device on a single-drop or multi-drop bus.
    /// Only the device that matches the registration will respond to the function command issued next;
    ///     all other devices will wait for a reset pulse.
    pub fn match_registration(&mut self, registration: Registration) -> OnewireResult<()> {
        self.reset().unwrap();
        self.write_byte(Commands::Match as u8);
        self.write_bytes(&u64::from(registration).to_le_bytes());
        Ok(())
    }

    /// Address all devices on the bus simultaneously.
    pub fn skip_registration(&mut self) -> OnewireResult<()> {
        self.reset().unwrap();
        self.write_byte(Commands::Skip as u8);
        Ok(())
    }
}

pub struct OnewireIterator<'a> {
    wire: &'a mut Onewire,
    command: SearchCommands,
    state: SearchAlgorithmState
}
impl <'a> Iterator for OnewireIterator<'a> {
    type Item = Registration;

    fn next(&mut self) -> Option<Self::Item> {
        if self.state.searching == Searching::Completed {
            return None;
        }
        loop {
            let r = self.wire.search(self.command, &mut self.state).unwrap();
            if r.is_some() {
                return r;
            }
        }
    }
}
