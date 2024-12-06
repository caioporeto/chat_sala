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
const struct device* stx = DEVICE_DT_GET(DT_NODELABEL(gpiob));

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 0

#define MSG_SIZE 8

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);


static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

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
	print_uart("Tell me something and press enter:\r\n");
}

char U = 0b01010101;
char sync = 0b00010110;
char STX = 0b00000010;
char id = 0b10101010;
char a[8];
char end = 0b00010111;

int i = 104;
int tx;

void TX(void){

	while(1){

		if(i>=96){
			k_msgq_get(&uart_msgq, &a, K_FOREVER); // pega as msgs da message queue
			i = 0;
		}

		if(i < 8){
			tx = (U >> (7-i)) & 0b1;
		}
		else if(i < 16){
			tx = (sync >> (7-(i%8))) & 0b1;
		}
		else if(i < 24){
			tx = (STX >> (7-(i%8))) & 0b1;
		}
		else if(i < 32){
			tx = (id >> (7-(i%8))) & 0b1;
		}
		else if(i < 88){
			tx = (a[(i/8)-4] >> (7-(i%8))) & 0b1; // seleciona cada caractere da string e shift para o lado e faz uma mascara para enviar apenas um bit
		}
		else if(i < 96){
			tx = (end >> (7-(i%8))) & 0b1;
		}

		gpio_pin_configure(stx, 0x3, GPIO_OUTPUT_ACTIVE);
		if(tx == 0b1){
			gpio_pin_set(stx, 0x3, 1);
			//printk("1");
		}
		else if(tx == 0b0){
			gpio_pin_set(stx, 0x3, 0);
			//printk("0");
		}

		i++;
		k_msleep(10);
	}
}

//K_TIMER_DEFINE(TX_id, TX, NULL);
//k_timer_start(TX_id, K_MSEC(100), K_MSEC(10));


K_THREAD_DEFINE(TX_id, MY_STACK_SIZE, TX, NULL, NULL, NULL, MY_PRIORITY, 0, 0); // Inicializar thread que executará F1
K_THREAD_DEFINE(leitura_teclado, MY_STACK_SIZE, ler_teclado, NULL, NULL, NULL, MY_PRIORITY, 0, 0); // Inicializar thread que executará F1