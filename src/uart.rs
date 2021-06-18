use core::cell::RefCell;
use stm32f4::stm32f407;
use lazy_static::lazy_static;
use cortex_m::interrupt::{self, Mutex};

lazy_static!{
	static ref STM_PERIPHERALS : Mutex<RefCell<stm32f407::Peripherals>>= {
		let p = stm32f407::Peripherals::take().unwrap();
		Mutex::new(RefCell::new(p))
	}; 
}

pub fn uart_init() {
	interrupt::free(|cs| {
		let p = STM_PERIPHERALS.borrow(cs).borrow();
		let uart = &p.USART2;

		// initializing gpio
		let gpioa = &p.GPIOA;
		let rcc = &p.RCC;

		// initializing clock
		rcc.ahb1enr.write(|w| w.gpioaen().bit(true));
		// initialize uart clock
		rcc.apb1enr.write(|w| w.usart2en().bit(true));

		// set up PA2 and PA3 pins for alternate function
		gpioa.afrl.modify(|_,w| w.afrl2().bits(0b0111).afrl3().bits(0b0111));
		gpioa.moder.modify(|_,w| w.moder2().bits(0b10).moder3().bits(0b10));

		// configure pin output speeds to high
		gpioa.ospeedr.modify(|_,w| w.ospeedr2().bits(0b10).ospeedr3().bits(0b10));

		// Enable the USART
		uart.cr1.modify(|_,w| w.ue().bit(true));

		// Set the word length
		uart.cr1.modify(|_,w| w.m().bit(false));

		// Program the number of stop bits
		uart.cr2.modify(|_,w| w.stop().bits(0));

		// Disable DMA transfer
		uart.cr3.modify(|_,w| w.dmat().bit(false));

		// Select the desired baudrate, in this case 16 MHz / 104.1875 = 9600 bits/second
		uart.brr.modify(|_,w| w.div_mantissa().bits(104).div_fraction().bits(3));

		// Initialize uart for reading and writing
		uart.cr1.modify(|_,w| w.te().bit(true).re().bit(true));
	})
}

pub fn uart_print_str(s: &str) {
	interrupt::free(|cs| {
		let p = STM_PERIPHERALS.borrow(cs).borrow();
		let uart = &p.USART2;
		for byte in s.as_bytes().iter() {
			while uart.sr.read().txe().bit_is_clear() {}

			uart.dr.write(|w| w.dr().bits(u16::from(*byte)));
		}
	})
}