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
#include <zephyr/random/random.h>
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
K_MUTEX_DEFINE(pode_transmitir);
K_MUTEX_DEFINE(mymutex);


K_CONDVAR_DEFINE(cond_RX_TX);
K_CONDVAR_DEFINE(cond_RX_Buffer);
K_CONDVAR_DEFINE(mycondvar);

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
K_MSGQ_DEFINE(msg_recebida, MSG_SIZE, 10, 4);

const struct device *pino_RX = DEVICE_DT_GET(GPIO_NODE);
static const struct gpio_dt_spec pino_TX = GPIO_DT_SPEC_GET(pin_tx, gpios);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


uint8_t Buffer = 0;
uint32_t recebido_RX = 0;
int reenvia = 0; // se algum byte enviado não for o esperado, reenviar a mensagem. Temos que implementar no codigo do TX uma forma de ele, toda vez que terminar de enviar o pacote, voltar essa variável para 0;
int startbit = 0; // variável auxiliar para sinalizar a leitura do startbit
int bit_RX;
int x = 0; //contador RX
int i = 0;
int k_bit = 97;
int tx;
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;
int mensagem_enviada = 1;
int tamanho;
int j=0;

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
static struct package pacote_enviado = {.U = 0b01010101, .sync = 0b00010110, .STX = 0b00000010, .id = 0b01010000, .end = 0b00010111};
char id_random = 0b10101001;

void montaBuffer(){

	for(int cont = 8; cont >= 1; cont--){
			
		Buffer = ((Buffer << 1) | ( (recebido_RX >> ((cont*4) - 3)) & 0b1)); // monta o buffer
	
	}

}

int bits = 0;
int p = 0;
int m = 0;
int pin; 

void RX (void){
    
	struct package *recebido = &pacote_recebido;
	struct package *enviado = &pacote_enviado;

	gpio_pin_configure(pino_RX, PIN_NUMBER, GPIO_INPUT); // define RX como input

	pin = !(gpio_pin_get(pino_RX , PIN_NUMBER));

	k_condvar_signal(&cond_RX_Buffer);

}

void RXBuffer(){
	struct package *recebido = &pacote_recebido;
	struct package *enviado = &pacote_enviado;

	while(1){

		k_condvar_wait(&cond_RX_Buffer, &mutex_RX_Buffer, K_FOREVER);

		recebido_RX = (recebido_RX << 1) | pin; 

		bits++;
		p++;

		printk("%d", pin);
		if(((recebido_RX >> 1) & 0b1) == ((recebido_RX >> 2) & 0b1) &&
		((recebido_RX >> 5) & 0b1) == ((recebido_RX >> 6) & 0b1) &&
		((recebido_RX >> 9) & 0b1) == ((recebido_RX >> 10) & 0b1) &&
		((recebido_RX >> 13) & 0b1) == ((recebido_RX >> 14) & 0b1) &&
		((recebido_RX >> 17) & 0b1) == ((recebido_RX >> 18) & 0b1) &&
		((recebido_RX >> 21) & 0b1) == ((recebido_RX >> 22) & 0b1) &&
		((recebido_RX >> 25) & 0b1) == ((recebido_RX >> 26) & 0b1) &&
		((recebido_RX >> 29) & 0b1) == ((recebido_RX >> 30) & 0b1)) { 
			//printk("byte valido");
		
			montaBuffer();

			if((Buffer == enviado->U) && (recebido->U != enviado->U)){
				recebido->U = Buffer;
				k_mutex_lock(&pode_transmitir, K_NO_WAIT);
				printk("U");
			
				p=0;
			}

			if(recebido->U == enviado->U){		
			
				if(((Buffer == enviado->sync) && (p==32) && (recebido->sync != enviado->sync))){
					recebido->sync = Buffer;
					printk("sync");
					p=0;
				}

				if(recebido->sync == enviado->sync){
					if((Buffer == enviado->STX && (p==32) && (recebido->STX != enviado->STX))){
						printk("STX");
						recebido->STX = Buffer;
						p=0;
					}

					if((recebido->STX == enviado->STX)){

						if((p==32) && (Buffer != enviado->id) && (recebido->id != enviado->id)){
							recebido->id = Buffer;
							recebido->n = recebido->id & 0b111;
							printk("id não nosso, %d %d ", recebido->id, recebido->n);
						}

						if((p>32) && (recebido->id != enviado->id)){
							//printk("entrou aqui sim");
							//printk("%c", Buffer);
							if(j < (recebido->n)){
								m++;
								if(m==16){
									recebido->a[j] = Buffer;
									//printk(" %c j%dj ", recebido->a[j], j);
									j++;
									m=0;
								}
							}
							
							if(j == ((recebido->n))){
									//printk("entrei aq");
									if((Buffer == enviado->end)){
										reenvia = 0;
										printk("end\n");
										printk("\nestamos recebendo certo!\nRecebemos: ");
										k_msgq_put(&msg_recebida, &(recebido->a), K_FOREVER);
										recebido->end = Buffer;
										recebido->U = 0;
										recebido->sync = 0;
										recebido->STX = 0;
										recebido->id = 0;
										recebido->end = 0;
										j=0;
										k_mutex_unlock(&pode_transmitir);
									}
								}
						}
						
						if(Buffer == enviado->id && (p==32) && (recebido->id != enviado->id)){
							recebido->id = Buffer;
							printk("id nosso");
							p=0;
						}

						if(recebido->id == enviado->id){
							
							recebido->n = (recebido->id) & 0b111;
							if((Buffer == enviado->a[0]) && (p==32) && (recebido->a[0] != enviado->a[0])){
								printk("%c", enviado->a[0]);
								recebido->a[0] = Buffer;
								p=0;
							}

							if(recebido->a[j] == enviado->a[j] && (p==32)){
								if((Buffer == enviado->a[(j+1)])){
									printk(" %c ", enviado->a[(j+1)]);
									recebido->a[(j+1)] = Buffer;
									j++;
									p=0;
								}
								if(j == ((enviado->n)-1)){
									printk("entrei aq");
									if((Buffer == enviado->end)){
										reenvia = 0;
										printk("end\n");
										printk("\nestamos transmitindo certo!\nRecebemos: ");
										k_msgq_put(&msg_recebida, &(recebido->a), K_FOREVER);
										recebido->end = Buffer;
										recebido->U = 0;
										recebido->sync = 0;
										recebido->STX = 0;
										recebido->id = 0;
										recebido->a[0] = 0;
										recebido->end = 0;
										j=0;
										k_mutex_unlock(&pode_transmitir);
									}
								}
							}
						}
					}
				}
			}
		}
	}
}


void seleciona_bit(int b){

	struct package *enviado = &pacote_enviado;
    
	if(b < 8){
		tx = (pacote_enviado.U >> (7-b)) & 0b1;
	}
	else if(b < 16){
		tx = (pacote_enviado.sync >> (7-(b%8))) & 0b1;
	}
	else if(b < 24){
		tx = (pacote_enviado.STX >> (7-(b%8))) & 0b1;
	}
	else if(b < 32){
		tx = (id_random >> (7-(b%8))) & 0b1;
	}
	else if(b < (32+(8*(enviado->n)))){
		tx = (pacote_enviado.a[(b/8)-4]) >> (7-(b%8)) & 0b1; // seleciona cada caractere da string e shift para o lado e faz uma mascara para enviar apenas um bit
	}
	else if(b <= (40+(8*(enviado->n)))){
		tx = (pacote_enviado.end >> (7-(b%8))) & 0b1;
	}

}

void serial_cb(const struct device *dev, void *user_data)
{
	struct package *enviar = &pacote_enviado;
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
			reenvia = 1;	
			enviar->n = rx_buf_pos;
			enviar->id = 0b01010000;
			enviar->id = ((enviar->id) | enviar->n);
			printk("\n\n\n%d %d\n\n\n", enviar->id, enviar->n);

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
	struct package *recebido = &pacote_recebido;
	struct package *enviado = &pacote_enviado;

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	uart_irq_rx_enable(uart_dev);

	print_uart("Hello! I'm your echo bot.\r\n");
	print_uart("Tell me something and press enter blabla:\r\n");

	while(1){
		if(reenvia == 0){
			k_msgq_get(&uart_msgq, &(pacote_enviado.a), K_FOREVER);
			printk("enviando...\n");
		}

		printk("\n");
		
		recebido->U = 0;
		recebido->sync = 0;
		recebido->STX = 0;
		recebido->id = 0;
		recebido->end = 0;
		j=0;
		p=0;

		int colisoes=0;
		int tempo=0;

		while(k_mutex_lock(&pode_transmitir, K_NO_WAIT) != 0){
			colisoes++;
			for(int h=0; h <= colisoes; h++){
				tempo += sys_rand32_get();
			}
			k_msleep(tempo);
		}

		for(k_bit = 0; k_bit < (40+(8*(enviado->n))+2); k_bit++){

			k_condvar_wait(&mycondvar, &mymutex, K_FOREVER);
			seleciona_bit(k_bit);
		
		}
		tx = 0;
		k_mutex_unlock(&pode_transmitir);
	}
}

void TX(void){

	k_condvar_signal(&mycondvar);
	gpio_pin_set_dt(&pino_TX, tx);
	
}

void print_tela(){

	struct package *recebido = &pacote_recebido;

	char msg[7];
	//printk("INICIALIZEI");

	while(1){
		if(k_msgq_get(&msg_recebida, &msg, K_FOREVER) == 0 ){
			//printk("ACHEI UMA MENSAGEM de %d ", recebido->n);
			for(int l = 0; l < (recebido->n); l++){
				printk("%c", msg[l]);
			}
			printk("\n");
		}
	}
}

K_THREAD_DEFINE(printar_na_tela, MY_STACK_SIZE, print_tela, NULL, NULL, NULL, 1, 0, 0);

K_THREAD_DEFINE(leitura_teclado, MY_STACK_SIZE, ler_teclado, NULL, NULL, NULL, MY_PRIORITY, 0, 0); // Inicializar thread que executará F1

K_THREAD_DEFINE(Buffer_RX, MY_STACK_SIZE, RXBuffer, NULL, NULL, NULL, MY_PRIORITY, 0, 0); // Inicializar thread que executará F1

K_TIMER_DEFINE(TX_timer, TX, NULL);

K_TIMER_DEFINE(RX_timer, RX, NULL);

int main(){
	int ret = gpio_pin_configure_dt(&pino_TX, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&pino_TX, 0);
	k_timer_start(&TX_timer, K_MSEC(10), K_MSEC(10));
	k_timer_start(&RX_timer, K_MSEC(10), K_USEC(2500));
}