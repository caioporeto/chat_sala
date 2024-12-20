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

K_MUTEX_DEFINE(mutex_RX_Buffer);
K_MUTEX_DEFINE(pode_transmitir);
K_MUTEX_DEFINE(mymutex);

K_CONDVAR_DEFINE(cond_RX_Buffer);
K_CONDVAR_DEFINE(mycondvar);

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
K_MSGQ_DEFINE(msg_recebida, MSG_SIZE, 10, 4);

const struct device *pino_RX = DEVICE_DT_GET(GPIO_NODE);
static const struct gpio_dt_spec pino_TX = GPIO_DT_SPEC_GET(pin_tx, gpios);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


uint8_t Buffer = 0; //buffer em que será alocado os bytes que forem válidos
uint32_t recebido_RX = 0; // lerá 32 bits (8*4)
int reenvia = 0; // se algum byte enviado não for o esperado, reenviar a mensagem. Temos que implementar no codigo do TX uma forma de ele, toda vez que terminar de enviar o pacote, voltar essa variável para 0;
int startbit = 0; // variável auxiliar para sinalizar a leitura do startbit
int bit_RX;
int x = 0; //contador RX
int i = 0;
int k_bit; 
int tx; //valor enviado ao pino tx
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

char id_random = 0b10101001; // id utilizado para testar quando receber mensagens de outra pessoa

void montaBuffer(){ // monta um buffer ao encontrar um byte valido

	for(int cont = 8; cont >= 1; cont--){
			
		Buffer = ((Buffer << 1) | ( (recebido_RX >> ((cont*4) - 3)) & 0b1)); // shifta o buffer 1 vez e faz a mascara com um dos bits do meio dos 4 lidos repetidamente
	
	}

}

int p = 0;
int m = 0;
int pin; 

void RX (void){
    
	struct package *recebido = &pacote_recebido;
	struct package *enviado = &pacote_enviado;

	gpio_pin_configure(pino_RX, PIN_NUMBER, GPIO_INPUT); // define o pino RX como input

	pin = !(gpio_pin_get(pino_RX , PIN_NUMBER)); // nega o valor recebido, pois recebe inverso do transmitido

	k_condvar_signal(&cond_RX_Buffer); // envia um sinal a thread para receber o bit lido (sincronização)

}

void RXBuffer(){
	struct package *recebido = &pacote_recebido;
	struct package *enviado = &pacote_enviado; 

	while(1){

		k_condvar_wait(&cond_RX_Buffer, &mutex_RX_Buffer, K_FOREVER); // espera um sinal do timer para receber o bit lido (sincronização)

		recebido_RX = (recebido_RX << 1) | pin; // shifta 1 pro lado e faz mascara com valor lidos

		p++;
 
		if(((recebido_RX >> 1) & 0b1) == ((recebido_RX >> 2) & 0b1) && // condição que verifica se recebeu um byte é valido
		((recebido_RX >> 5) & 0b1) == ((recebido_RX >> 6) & 0b1) &&
		((recebido_RX >> 9) & 0b1) == ((recebido_RX >> 10) & 0b1) &&
		((recebido_RX >> 13) & 0b1) == ((recebido_RX >> 14) & 0b1) &&
		((recebido_RX >> 17) & 0b1) == ((recebido_RX >> 18) & 0b1) &&
		((recebido_RX >> 21) & 0b1) == ((recebido_RX >> 22) & 0b1) &&
		((recebido_RX >> 25) & 0b1) == ((recebido_RX >> 26) & 0b1) &&
		((recebido_RX >> 29) & 0b1) == ((recebido_RX >> 30) & 0b1)) { 
		
			montaBuffer(); // monta o buffer com os bits recebidos 

			if((Buffer == enviado->U) && (recebido->U != enviado->U)){ // verifica se o buffer é igual ao pre e se ainda nao foi recebido o pre 
				
				recebido->U = Buffer; 

				k_mutex_lock(&pode_transmitir, K_NO_WAIT); //tenta travar o mutex para caso alguem esteja transmitindo	
				
				p=0; 
			}

			if(recebido->U == enviado->U){ //verifica se ja recebeu o pre
			
				if(((Buffer == enviado->sync) && (p==32) && (recebido->sync != enviado->sync))){ //verifica se o proximo byte pós pre lido é igual ao sync
					recebido->sync = Buffer;
					p=0;
				}

				if(recebido->sync == enviado->sync){
					if((Buffer == enviado->STX && (p==32) && (recebido->STX != enviado->STX))){ //verifica se o proximo byte pós sync lido é igual ao STX
						recebido->STX = Buffer;
						p=0;
					}

					if((recebido->STX == enviado->STX)){

						if((p==32) && (Buffer != enviado->id) && (recebido->id != enviado->id)){ //verifica se o cabeçalho possui nosso id
							recebido->id = Buffer; //recebe o id de outrém
							recebido->n = recebido->id & 0b111; //define que o numero de bits que serão enviados
						}

						if((p>32) && (recebido->id != enviado->id)){ //pega os proximos bits a partir do 32°s
					
							if(j < (recebido->n)){ // entra enquanto todas os caracteres nao tiverem sido recebidos
								m++;
								if(m==16){ // pega os proximos 32 bits lidos (m == 16 pois a contagem é feita apenas pelos bytes validos)
									recebido->a[j] = Buffer; // preenche com o caractere recebido
									j++;
									m=0;
								}
							}
							
							if(j == ((recebido->n))){ // caso ja tenha lido os n caracteres

									if((Buffer == enviado->end)){ //
										reenvia = 0; // reenviar não é mais necessário 
										printk("\nestamos recebendo certo!\nRecebemos: ");
										k_msgq_put(&msg_recebida, &(recebido->a), K_FOREVER);
										recebido->end = Buffer;
										recebido->U = 0; // zera tudo recebido no protocolo
										recebido->sync = 0; 
										recebido->STX = 0;  
										recebido->id = 0;
										recebido->a[0] = 0;
										recebido->end = 0; 
										j=0;
										k_mutex_unlock(&pode_transmitir); // desbloqueia mutex para poder transmitir
									}
								}
						}
						
						if(Buffer == enviado->id && (p==32) && (recebido->id != enviado->id)){ // verifica se o id é nosso (caso nós queiramos conferir nossa transmissão)
							recebido->id = Buffer;
							p=0;
						}

						if(recebido->id == enviado->id){ 
							
							recebido->n = (recebido->id) & 0b111; // define o numero de bits que serão enviados
							if((Buffer == enviado->a[0]) && (p==32) && (recebido->a[0] != enviado->a[0])){ //verifica se o proximo byte pós cabeçalho é igual ao primeiro caractere enviado 
								recebido->a[0] = Buffer;
								p=0;
							}

							if(recebido->a[j] == enviado->a[j] && (p==32)){
								if((Buffer == enviado->a[(j+1)])){ //verifica se o proximo byte pós caractere é igual ao proximo caractere enviado 
									recebido->a[(j+1)] = Buffer;
									j++;
									p=0;
								}

								if(j == ((recebido->n)-1)){ // verifica se todos os bits já foram recebidos
									if((Buffer == enviado->end)){ //verifica se o proximo byte pós ultimo caractere é igual ao end 
										reenvia = 0; // não é mais necessário reenviar
										printk("\nestamos transmitindo certo!\nRecebemos: ");
										k_msgq_put(&msg_recebida, &(recebido->a), K_FOREVER); // coloca mensagem na message queue para printar na serial
										recebido->end = Buffer;
										recebido->U = 0; //zera tudo recebido no protocolo
										recebido->sync = 0;
										recebido->STX = 0;
										recebido->id = 0;
										recebido->a[0] = 0;
										recebido->end = 0;
										j=0;
										k_mutex_unlock(&pode_transmitir); // libera mutex para transmitir
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
    
	if(b < 8){ //os 8 primeiros bits são do U 
		tx = (pacote_enviado.U >> (7-b)) & 0b1;
	}
	else if(b < 16){ //os 8 segundos bits são do sync
		tx = (pacote_enviado.sync >> (7-(b%8))) & 0b1;
	}
	else if(b < 24){ //os 8 terceiros bits são do STX
		tx = (pacote_enviado.STX >> (7-(b%8))) & 0b1;
	}
	else if(b < 32){ //os 8 quartos bits são do id
		tx = (pacote_enviado.id >> (7-(b%8))) & 0b1;
	}
	else if(b < (32+(8*(enviado->n)))){ //os proximos bits são dos caracteres
		tx = (pacote_enviado.a[(b/8)-4]) >> (7-(b%8)) & 0b1; // seleciona cada caractere da string e shift para o lado e faz uma mascara para enviar apenas um bit
	}
	else if(b <= (40+(8*(enviado->n)))){ //os 8 ultimos bits são do end
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

	/* lê até que a FIFO esteja vazia */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* encerra string */
			rx_buf[rx_buf_pos] = '\0';

			/* coloca mensagem na fila para ser transmitida */
			k_msgq_put(&uart_msgq, &rx_buf, K_FOREVER);	
			reenvia = 1;	
			enviar->n = rx_buf_pos; //define a quantidade de caracteres
			enviar->id = 0b01010000; //define o id no cabeçalho
			enviar->id = ((enviar->id) | enviar->n); // define todo o cabeçalho

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
			k_msgq_get(&uart_msgq, &(pacote_enviado.a), K_FOREVER); // aguarda na fila até que tenha alguma mensagem para transmitir
			printk("enviando...\n");
		}

		printk("\n");
		
		recebido->U = 0; // zera tudo para que ter certeza que o pacote foi enviado corretamente de uma vez só 
		recebido->sync = 0;
		recebido->STX = 0;
		recebido->id = 0;
		recebido->end = 0;
		j=0;
		p=0;

		int colisoes=0;
		int tempo=0;

		while(k_mutex_lock(&pode_transmitir, K_NO_WAIT) != 0){ // caso haja colisoes, espera um tempo indeterminado para transmitir
			
			colisoes++; // soma a quantidade de "colisões"

			for(int h=0; h <= colisoes; h++){
				tempo += sys_rand32_get(); // soma um tempo aleatorio 
			}

			k_msleep(tempo);
		}

		for(k_bit = 0; k_bit < (40+(8*(enviado->n))+2); k_bit++){ // envia todos os bits do pacote

			k_condvar_wait(&mycondvar, &mymutex, K_FOREVER); // espera um sinal para que possa alterar o valor de tx
			seleciona_bit(k_bit); // altera o valor de tx para o proximo bit a ser enviado
		
		}
		tx = 0;
		k_mutex_unlock(&pode_transmitir); // desbloqueia o mutex para caso ja tenha terminado de transmitir
	}
}

void TX(void){

	k_condvar_signal(&mycondvar); // dá sinal para alterar o valor de tx
	gpio_pin_set_dt(&pino_TX, tx); // seta o pino TX para o valor tx
	
}

void print_tela(){

	struct package *recebido = &pacote_recebido;

	char msg[7];

	while(1){
		if(k_msgq_get(&msg_recebida, &msg, K_FOREVER) == 0 ){ // espera para quando houver uma mensagem que foi recebida

			for(int l = 0; l < (recebido->n); l++){ //printa os caracteres da mensagem em seguida
				printk("%c", msg[l]); 
			}
			printk("\n");
		}
	}
}

K_THREAD_DEFINE(printar_na_tela, MY_STACK_SIZE, print_tela, NULL, NULL, NULL, 1, 0, 0);

K_THREAD_DEFINE(leitura_teclado, MY_STACK_SIZE, ler_teclado, NULL, NULL, NULL, MY_PRIORITY, 0, 0);

K_THREAD_DEFINE(Buffer_RX, MY_STACK_SIZE, RXBuffer, NULL, NULL, NULL, MY_PRIORITY, 0, 0);

K_TIMER_DEFINE(TX_timer, TX, NULL);

K_TIMER_DEFINE(RX_timer, RX, NULL);

int main(){
	int ret = gpio_pin_configure_dt(&pino_TX, GPIO_OUTPUT_ACTIVE); // configura o pino TX para output
	gpio_pin_set_dt(&pino_TX, 0); // seta o pino inicialmente para zero
	k_timer_start(&TX_timer, K_MSEC(10), K_MSEC(10)); // começa o timer para chamar a callback a cada 10ms (100bps)
	k_timer_start(&RX_timer, K_MSEC(10), K_USEC(2500)); // começa o timer para chamar a callback a cada 2.5ms (400bps)
}