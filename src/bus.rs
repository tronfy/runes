pub struct Bus {
    pub ram: [u8; 64 * 1024],
}

impl Bus {
    pub fn new() -> Bus {
        Bus {
            ram: [0; 64 * 1024],
        }
    }

    pub fn write(&mut self, address: u16, data: u8) {
        if address >= 0x0000 && address <= 0xFFFF {
            self.ram[address as usize] = data;
        }
    }

    pub fn read(&self, address: u16) -> u8 {
        if address >= 0x0000 && address <= 0xFFFF {
            return self.ram[address as usize];
        }
        0
    }
}
