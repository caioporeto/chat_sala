#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define PIN_NUMBER 3 // Substitua pelo número do pino
#define GPIO_NODE DT_NODELABEL(gpiob) // Controlador GPIO (troque por "gpioa" ou outro conforme necessário)

const struct device *pino_RX = DEVICE_DT_GET(GPIO_NODE);

int buffer;
int bit_RX;

void RX(void){
	gpio_pin_configure(pino_RX, PIN_NUMBER, GPIO_INPUT);
	bit_RX = gpio_pin_get(pino_RX , PIN_NUMBER);

	buffer = ((buffer << 1) | bit_RX) & 0b1111;	
}

K_TIMER_DEFINE(RX_timer, RX, NULL);

int main(){
	
	k_timer_start(&RX_timer, K_USEC(2500), K_USEC(2500));
	k_timer_start(&TX_timer, K_MSEC(10), K_MSEC(10));

}