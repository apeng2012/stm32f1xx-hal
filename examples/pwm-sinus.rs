#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _;

use core::cell::RefCell;
use core::f32::consts::FRAC_PI_2;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use micromath::F32Ext;
use stm32f1xx_hal::{
    gpio::{Alternate, Pin},
    pac::{self, interrupt, Interrupt, TIM2},
    prelude::*,
    timer::{self, pwm::PwmHz, Ch, Channel, Tim2NoRemap},
};

const N: usize = 50;

type PwmC1 = PwmHz<TIM2, Tim2NoRemap, Ch<0>, Pin<'A', 0, Alternate>>;
type SinA = [u16; N + 1];

static G_PWM: Mutex<RefCell<Option<PwmC1>>> = Mutex::new(RefCell::new(None));
static G_SINA: Mutex<RefCell<Option<SinA>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let Some(dp) = pac::Peripherals::take() {
        // Set up the system clock.
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).freeze(&mut flash.acr);

        let mut afio = dp.AFIO.constrain();

        let mut gpioa = dp.GPIOA.split();

        // TIM2
        let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);

        let mut pwm = dp
            .TIM2
            .pwm_hz::<Tim2NoRemap, _, _>(c1, &mut afio.mapr, 1.kHz(), &clocks);

        let max_duty = pwm.get_max_duty();

        let mut sin_a: SinA = [0_u16; N + 1];
        // Fill sinus array
        let a = FRAC_PI_2 / (N as f32);
        for (i, b) in sin_a.iter_mut().enumerate() {
            let angle = a * (i as f32);
            *b = (angle.sin() * (max_duty as f32)) as u16;
        }
        cortex_m::interrupt::free(|cs| *G_SINA.borrow(cs).borrow_mut() = Some(sin_a));

        pwm.enable(Channel::C1);
        pwm.listen(timer::Event::Update);
        cortex_m::interrupt::free(|cs| *G_PWM.borrow(cs).borrow_mut() = Some(pwm));

        unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
        }
    }

    loop {
        cortex_m::asm::nop();
    }
}

#[interrupt]
fn TIM2() {
    static mut I: usize = 0;
    static mut PWM: Option<PwmC1> = None;
    static mut SINA: Option<SinA> = None;

    let i = {
        *I += 1;
        if *I >= 4 * N {
            *I = 0;
            0
        } else {
            *I
        }
    };

    let pwm = PWM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_PWM.borrow(cs).replace(None).unwrap())
    });

    let sin_a = SINA.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_SINA.borrow(cs).replace(None).unwrap())
    });

    pwm.clear_interrupt(timer::Event::Update);

    if i == 0 || i == 2 * N {
        pwm.set_duty(Channel::C1, 0);
    } else if i < N {
        pwm.set_duty(Channel::C1, sin_a[i]);
    } else if i < 2 * N {
        pwm.set_duty(Channel::C1, sin_a[2 * N - i]);
    } else if i < 3 * N {
        pwm.set_duty(Channel::C1, sin_a[i - 2 * N]);
    } else {
        pwm.set_duty(Channel::C1, sin_a[4 * N - i]);
    }
}
