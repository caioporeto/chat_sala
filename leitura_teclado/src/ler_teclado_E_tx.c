/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define PIN_NUMBER 3 // Substitua pelo número do pino
#define GPIO_NODE DT_NODELABEL(gpiob) // Controlador GPIO (troque por "gpioa" ou outro conforme necessário)
#define pin_tx DT_ALIAS(sw0)

const struct device *pino_RX = DEVICE_DT_GET(GPIO_NODE);
static const struct gpio_dt_spec pino_TX = GPIO_DT_SPEC_GET(pin_tx, gpios);

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 0

#define MSG_SIZE 8

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;
int mensagem_enviada = 1;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;
	//k_mutex_lock(&mutexzinho, K_FOREVER);
	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
			mensagem_enviada = 0;

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

void ler_teclado(void){
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		//return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
	}
	uart_irq_rx_enable(uart_dev);

	print_uart("Hello! I'm your echo bot.\r\n");
	print_uart("Tell me something and press enter blabla:\r\n");
}

struct package {
	char U;
	char sync;
	char STX;
	char id;
	char end;
	char a[8];
};

int i = 96;
int tx;
struct package pacote;

void TX(void){

	gpio_pin_configure(pino_RX, PIN_NUMBER, GPIO_INPUT); // define RX como input

	pacote.U = 0b01010101;
	pacote.sync = 0b00010110;
	pacote.STX = 0b00000010;
	pacote.id = 0b01010111;
	pacote.end = 0b00010111;

	if(mensagem_enviada == 0){
	if(i>=96){
	 	// pega as msgs da message queue
		k_msgq_get(&uart_msgq, &(pacote.a), K_FOREVER);
		i = 0;
	}

	if(i < 8){
		tx = (pacote.U >> (7-i)) & 0b1;
	}
	else if(i < 16){
		tx = (pacote.sync >> (7-(i%8))) & 0b1;
	}
	else if(i < 24){
		tx = (pacote.STX >> (7-(i%8))) & 0b1;
	}
	else if(i < 32){
		tx = (pacote.id >> (7-(i%8))) & 0b1;
	}
	else if(i < 88){
		tx = (pacote.a[(i/8)-4]) >> (7-(i%8)) & 0b1; // seleciona cada caractere da string e shift para o lado e faz uma mascara para enviar apenas um bit
	}
	else if(i < 96){
		tx = (pacote.end >> (7-(i%8))) & 0b1;
	}
	gpio_pin_set_dt(&pino_TX, tx);

	/*
	int bit_RX = !(gpio_pin_get(pino_RX , PIN_NUMBER));


	if(bit_RX == 0b1){
		printk("1");
	}
	else if(bit_RX == 0b0){
		printk("0");
	}
	*/

	i++;
	if(i>=96){
		//printk("\n");
		mensagem_enviada=1;
	}
	}
}

K_THREAD_DEFINE(leitura_teclado, MY_STACK_SIZE, ler_teclado, NULL, NULL, NULL, MY_PRIORITY, 0, 0); // Inicializar thread que executará F1

K_TIMER_DEFINE(TX_timer, TX, NULL);

int main(){
	int ret = gpio_pin_configure_dt(&pino_TX, GPIO_OUTPUT_ACTIVE);
	k_timer_start(&TX_timer, K_MSEC(10), K_MSEC(10));
}