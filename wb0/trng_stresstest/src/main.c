/**
 * Does entropy_stm32_rng_get_entropy_isr work with WB09 when TRNG interrupt is active? Have you tested it?
 * 
 * Answer this using CODE!
 * 
 * 1) Configure GPIO pin "PX" as input & output, and attach event handler
 * 2) Trigger GPIO interrupt on "PX" to have event handler invoked
 * 3) Invoke entropy_stm32_rng_get_entropy_isr from GPIO event handler
 * 4) Trigger NMI using a hook inside entropy_stm32_rng_get_entropy_isr
 * 5) Invoke entropy_stm32_rng_get_entropy_isr from NMI handler
 * 6) ?
 * 7) Profit
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/arch/arm/nmi.h>

// Configuration options
#define GPIO_PORT gpiob
#define GPIO_PIN 15
#define WAIT_POOLS_FULL 1

/* ------------------------- */

const struct device *trng = DEVICE_DT_GET(DT_NODELABEL(rng));
const struct device *gpio_port = DEVICE_DT_GET(DT_NODELABEL(GPIO_PORT));

#define BUFLEN 256
uint8_t regular_buffer[BUFLEN];
uint8_t isr_buffer[BUFLEN];
uint8_t nmi_buffer[BUFLEN];

struct gpio_callback gpiocb_block;

static void printbuffer(const uint8_t *buf, int len)
{
	for (int i = 0; i < len; i++) {
		printk(" %02hhX", buf[i]);

		if (((i + 1) % 16) == 0) {
			printk("\n");
		}
	}
	printk("\n");
}

/* ------------------------- */

static void custom_gpio_isr(const struct device *port,
		struct gpio_callback *cb,
		gpio_port_pins_t pins)
{
	/* 3) Invoke entropy driver from GPIO ISR */
	int ret = entropy_get_entropy_isr(trng, isr_buffer, sizeof(isr_buffer), ENTROPY_BUSYWAIT);

	printk("%s: ret=%d\n", __func__, ret);
	if (ret >= 0) {
		printbuffer(isr_buffer, sizeof(isr_buffer));
	}
}

/* Driver-side hook */
void __trng_backdoor(void)
{
	static bool enter_nmi_on_backdoor = true;

	if (enter_nmi_on_backdoor) {
		/* Make sure next invokation doesn't trigger NMI */
                enter_nmi_on_backdoor = false;

		printk("[i] Triggering NMI\n");

		/* 4) Trigger NMI from entropy driver */
                SCB->ICSR |= SCB_ICSR_NMIPENDSET_Msk;
        }
}

static void custom_nmi_isr(void)
{
	/* 5) Invoke entropy driver from NMI which suspended a call */
	int ret = entropy_get_entropy_isr(trng, nmi_buffer, sizeof(nmi_buffer), ENTROPY_BUSYWAIT);

	printk("%s: ret=%d\n", __func__, ret);
	if (ret >= 0) {
		printbuffer(nmi_buffer, sizeof(nmi_buffer));
	}
}

int main(void) {
	int ret;

	/* 0) Install custom NMI handler */
	z_arm_nmi_set_handler(custom_nmi_isr);
	printk("[+] Installed custom NMI handler\n");

	/* 1) Configure GPIO pin */
	ret = gpio_pin_configure(gpio_port, GPIO_PIN, 
		GPIO_OUTPUT_LOW | GPIO_OUTPUT | GPIO_INPUT);
	if (ret < 0) {
		printk("[-] gpio_pin_configure failed %d\n", ret);
		return ret;
	}

	gpio_init_callback(&gpiocb_block, custom_gpio_isr, BIT(GPIO_PIN));
	ret = gpio_add_callback(gpio_port, &gpiocb_block);
	if (ret < 0) {
		printk("[-] gpio_add_callback failed %d\n", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure(gpio_port, GPIO_PIN, GPIO_INT_EDGE_RISING);
	if (ret < 0) {
		printk("[-] gpio_pin_interrupt_configure failed %d\n", ret);
		return ret;
	}
	printk("[+] Configured GPIO pin\n");

#if WAIT_POOLS_FULL
	/* Wait until RNG pools are filled and hardware is turned off */
	printk("[i] Sleeping for 5 seconds...\n");
	k_msleep(5000);
#endif 

#if defined(CONFIG_SOC_SERIES_STM32WB0X)
	printk("RNG clock enabled? %d\n", LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_RNG));
#endif

	/* 2) Trigger GPIO interrupt */
	ret = gpio_pin_set(gpio_port, GPIO_PIN, 1);
	if (ret < 0) {
		printk("[-] gpio_port_set failed %d\n", ret);
		return ret;
	}
	printk("[+] Triggered GPIO interrupt\n");

        /* something should happen anytime soon... */
	printk("FINI\n");

        return 0;
}
