#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define PIN_NUMBER 3 // Substitua pelo número do pino
#define GPIO_NODE DT_NODELABEL(gpiob) // Controlador GPIO (troque por "gpioa" ou outro conforme necessário)
#define pin_tx DT_ALIAS(sw0)

const struct device *pino_RX = DEVICE_DT_GET(GPIO_NODE);
static const struct gpio_dt_spec pino_TX = GPIO_DT_SPEC_GET(pin_tx, gpios);

int buffer;
int bit_RX;

void RX(void){
	gpio_pin_configure(pino_RX, PIN_NUMBER, GPIO_INPUT); // define RX como input
	
	gpio_pin_set_dt(&pino_TX, 0);
	
	bit_RX = gpio_pin_get(pino_RX , PIN_NUMBER);

	buffer = ((buffer << 1) | bit_RX) & 0b1111;	
	if(bit_RX == 0b1){
		printk("1");
	}
	
	else if(bit_RX == 0b0){
		printk("0");
	}

}

K_TIMER_DEFINE(RX_timer, RX, NULL);

int main(){
	
	k_timer_start(&RX_timer, K_USEC(2500), K_USEC(2500));
	int ret = gpio_pin_configure_dt(&pino_TX, GPIO_OUTPUT_ACTIVE);

}