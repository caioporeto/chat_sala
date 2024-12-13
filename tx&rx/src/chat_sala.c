/*
 * Copyright SydCaio
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define PIN_NUMBER 3 // Substitua pelo número do pino
#define GPIO_NODE DT_NODELABEL(gpiob) // Controlador GPIO (troque por "gpioa" ou outro conforme necessário)
#define pin_tx DT_ALIAS(sw0)
#define MY_STACK_SIZE 1024
#define MY_PRIORITY 0
#define MSG_SIZE 8

K_MUTEX_DEFINE(mutex_TX_RX);
K_MUTEX_DEFINE(mutex_RX_Buffer);
K_MUTEX_DEFINE(mutex_ninguem_esta_transmitindo);

K_CONDVAR_DEFINE(cond_RX_TX);
K_CONDVAR_DEFINE(cond_RX_Buffer);

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
K_MSGQ_DEFINE(msg_recebida, MSG_SIZE, 10, 4);

const struct device *pino_RX = DEVICE_DT_GET(GPIO_NODE);
static const struct gpio_dt_spec pino_TX = GPIO_DT_SPEC_GET(pin_tx, gpios);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


uint32_t Buffer = 0;
uint32_t recebido_RX = 0;
int reenviar = 0; // se algum byte enviado não for o esperado, reenviar a mensagem. Temos que implementar no codigo do TX uma forma de ele, toda vez que terminar de enviar o pacote, voltar essa variável para 0;
int startbit = 0; // variável auxiliar para sinalizar a leitura do startbit
int bit_RX;
int x = 0; //contador RX
int i = 0;
int k_bit = 96;
int tx;
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;
int mensagem_enviada = 1;

struct package {
	char bitstart;
	char U;
	char sync;
	char STX;
	char id;
	char end;
	char a[8];
	char n;
};

static struct package pacote_recebido;
static struct package pacote_enviado;


void RX (void){
    
	struct package *recebido = &pacote_recebido;
	struct package *enviado = &pacote_enviado;

	static int id;
    static int i;
    static int j;
    static int n = 0;
    static int v = 0;
	static int cont = 0;
	static int cont1 = 0;
	static int cont2 = 0;

	gpio_pin_configure(pino_RX, PIN_NUMBER, GPIO_INPUT); // define RX como input

	int pin = !(gpio_pin_get(pino_RX , PIN_NUMBER));
	
	recebido_RX = (recebido_RX << 1) | pin; 
	printk("%d %d", pin, recebido_RX);

	if(((recebido_RX >> 1) & 0b1) == ((recebido_RX >> 2) & 0b1) &&
	((recebido_RX >> 5) & 0b1) == ((recebido_RX >> 6) & 0b1) &&
	((recebido_RX >> 9) & 0b1) == ((recebido_RX >> 10) & 0b1) &&
	((recebido_RX >> 13) & 0b1) == ((recebido_RX >> 14) & 0b1) &&
	((recebido_RX >> 17) & 0b1) == ((recebido_RX >> 18) & 0b1) &&
	((recebido_RX >> 21) & 0b1) == ((recebido_RX >> 22) & 0b1) &&
	((recebido_RX >> 25) & 0b1) == ((recebido_RX >> 26) & 0b1) &&
	((recebido_RX >> 29) & 0b1) == ((recebido_RX >> 30) & 0b1)) { 
	//printk("byte valido");
	// se for verdade, é um byte valido
	for(cont = 8; cont >= 1; cont--){
		Buffer = (Buffer & ((recebido_RX & 0b1) >> ((cont*4) - 3))); // monta o buffer
		if(cont == 1){
			printk(" %d\n", Buffer);
		}
	}
	
	/*
				printk("%d %d\n", Buffer, recebido_RX);
				if (Buffer == enviado->U){
					recebido->U = Buffer;
					printk("U\n");
					for (cont1 = 0; cont1 < 32; cont1++){
						recebido_RX = ((recebido_RX << 1) | (gpio_pin_get(pino_RX , PIN_NUMBER)));
					}
					for(cont = 8; cont >= 1; cont--){
						Buffer = (Buffer & (recebido_RX >> (cont*4) - 3));
					}
					if (Buffer == enviado->sync) {
						
						recebido->sync = Buffer;
						//printk("sync\n");
						for (cont1 = 0; cont1 < 32; cont1++){
							recebido_RX = ((recebido_RX << 1) | (gpio_pin_get(pino_RX , PIN_NUMBER)));
						}
						for(cont = 8; cont >= 1; cont--){
						Buffer = (Buffer & (recebido_RX >> (cont*4) - 3));
							}

						recebido->STX = Buffer;
						//printk("STX\n");
						for (cont1 = 0; cont1 < 32; cont1++){
							recebido_RX = ((recebido_RX << 1) | (gpio_pin_get(pino_RX , PIN_NUMBER)));
						}
						for(cont = 8; cont >= 1; cont--){
							Buffer = (Buffer & (recebido_RX >> (cont*4) - 3));
						}

						id = (Buffer & 0b00011111); // guarda os 5 primeiros bits para obter o id
						n = Buffer;
						n = ((n >> 5) & 0b00000111); // shifta 5 para direita para obter os 3 ultimos bits, o n// se for o mesmo id que você, sinalizar que não é necessário printar na tela. Caso contrário apenas armazenar e pedir para reenviar
						recebido->id = id;
						recebido->n = n;
						//printk("ID\n");
						for(cont2; cont2 < n; cont2++){
							for (cont1 = 0; cont1 < 32; cont1++){
								recebido_RX = ((recebido_RX << 1) | (gpio_pin_get(pino_RX , PIN_NUMBER)));
							}
							for(cont = 8; cont >= 1; cont--){
								Buffer = (Buffer & (recebido_RX >> (cont*4) - 3));
							}
							recebido->a[v++] = Buffer;
						}
						//printk("poretop\n");

						for(cont = 8; cont >= 1; cont--){
							Buffer = (Buffer & (recebido_RX >> (cont*4) - 3));
						}
						recebido->end = Buffer;
						//printk("end\n");
						
						if ((recebido->STX == enviado->STX) &&
						(recebido->n == enviado->n) &&
						(recebido->id == enviado->id) &&
						(memcmp(recebido->a, enviado->a, sizeof(recebido->a) == 0)) &&
						(recebido->end == enviado->end)) {
							reenviar = 0;
							printk("ok");
						} 
						else {
							reenviar = 1;
							printk("reenviar");
						}

					}
					else{
						reenviar = 1;
					}
				}
				else{
				}*/
		}
	}

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
			k_msgq_put(&uart_msgq, &rx_buf, K_FOREVER);
			mensagem_enviada = 0;

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int z = 0; z < msg_len; z++) {
		uart_poll_out(uart_dev, buf[z]);
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

void TX(void){

	struct package *enviar = &pacote_enviado;

	enviar->U = 0b01010101;
	enviar->sync = 0b00010110;
	enviar->STX = 0b00000010;
	enviar->id = 0b01010111;
	enviar->end = 0b00010111;

	if(mensagem_enviada==0){

		//printk("SID");
	if(k_bit>=96){
        //k_mutex_lock(&mutex_ninguem_esta_transmitindo, K_FOREVER); // só vai começar a enviar caso nao esteja recebendo nada no RX
		//printk("TAJ MAHAL");
		k_msgq_get(&uart_msgq, &(pacote_enviado.a), K_FOREVER);
	//if (reenviar == 0){ // pega as mensagens da message queue, mas apenas se a mensagem previamente enviada tiver sido recebida
		
	
		k_bit = 0;
		printk("enviando");
	}
	
	if(k_bit < 8){
		tx = (pacote_enviado.U >> (7-k_bit)) & 0b1;
	}
	else if(k_bit < 16){
		tx = (pacote_enviado.sync >> (7-(k_bit%8))) & 0b1;
	}
	else if(k_bit < 24){
		tx = (pacote_enviado.STX >> (7-(k_bit%8))) & 0b1;
	}
	else if(k_bit < 32){
		tx = (pacote_enviado.id >> (7-(k_bit%8))) & 0b1;
	}
	else if(k_bit < 88){
		tx = (pacote_enviado.a[(k_bit/8)-4]) >> (7-(k_bit%8)) & 0b1; // seleciona cada caractere da string e shift para o lado e faz uma mascara para enviar apenas um bit
	}
	else if(k_bit < 96){
		tx = (pacote_enviado.end >> (7-(k_bit%8))) & 0b1;
	}
	gpio_pin_set_dt(&pino_TX, tx);


	k_bit++;
	if(k_bit>=96){
		mensagem_enviada = 1;
        //reenviar = 1;
		gpio_pin_set_dt(&pino_TX, 0); // Voltar variável reenviar para 1 para prosseguir para o proximo elemento da msgq caso a mensagem seja recebida corretamente pela RX_Buffer
        //k_mutex_unlock(&mutex_ninguem_esta_transmitindo); // finalizada a transmissão, liberar mutex
	}
	}
}

void print_tela(void *arg1){

	struct package *recebido = (struct package *)arg1;

	char msg[7];

	while(1){
		if(k_msgq_get(&msg_recebida, &msg, K_FOREVER) == 0 ){
			for(int l = 0; l < recebido->n; l++){
				printk("%c", msg[l]);
			}
		}
	}
}

//K_THREAD_DEFINE(RX_Buffer_id, MY_STACK_SIZE, RX_Buffer, &pacote_recebido, &pacote_enviado, NULL, 1, 0, 10);

K_THREAD_DEFINE(printar_na_tela, MY_STACK_SIZE, print_tela, &pacote_recebido, NULL, NULL, 1, 0, 0);

K_THREAD_DEFINE(leitura_teclado, MY_STACK_SIZE, ler_teclado, NULL, NULL, NULL, MY_PRIORITY, 0, 0); // Inicializar thread que executará F1

//K_THREAD_DEFINE(rxz, MY_STACK_SIZE, RX, NULL, NULL, NULL, 1, 0, 0); // Inicializar thread que executará F1

K_TIMER_DEFINE(TX_timer, TX, NULL);

K_TIMER_DEFINE(RX_timer, RX, NULL);

int main(){
	int ret = gpio_pin_configure_dt(&pino_TX, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&pino_TX, 0);
	k_timer_start(&TX_timer, K_MSEC(10), K_MSEC(10));
	k_timer_start(&RX_timer, K_USEC(2500), K_USEC(2500));
	
}