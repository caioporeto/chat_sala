#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 0

#define SW0_NODE	DT_ALIAS(sw0)

K_FIFO_DEFINE(FIFO_TX);
K_FIFO_DEFINE(FIFO_RX);

//K_MUTEX_DEFINE(mutex_get_FIFO);
K_MUTEX_DEFINE(mutex_ninguem_esta_enviando); // mutex para bloquear TX caso alguem esteje enviando uma mensagem
K_MUTEX_DEFINE(mutex_envio); // mutex para as funções TX e RX_proprio trabalharem alternadamente

struct data_item_t {
    void *fifo_reserved;
    char chave[100]; // ID + N + CHAR
}

static const struct gpio_dt_spec TX_pin = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});

int envio_correto = 1; // se for 1, enviou corretamente, se for 0 enviou incorretamente
int tempo_aleatório = 10;
int sync = 0b00000000;
int stx = 0b00000000;
int end = 0b00000000;
int id_proprio = 0b00000000;

void TX (void) {
    int mensagem;
    int ret;

	if (!gpio_is_ready_dt(&TX_pin)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&TX_pin, GPIO_OUTPUT);
	if (ret != 0) {
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&TX_pin, GPIO_OUTPUT);
	if (ret != 0) {
		return 0;
	}

    while (1) { // substituir por um timer depois
    
        if((k_mutex_lock(&mutex_envio, K_FOREVER)) == 0){
            if (envio_correto == 1) { // se o envio foi correto, enviar a próxima mensagem da FIFO
                struct data_item_t *data_teclado = k_fifo_get(&FIFO_TX, K_FOREVER);
                mensagem = data_teclado->chave;
            }
            else if (envio_correto == 0){ // se o envio foi incorreto, enviar a mesma mensagem

            }
            if((k_mutex_lock(&mutex_ninguem_esta_enviando, K_FOREVER)) == 0){
                Tx_pin = U; 
                Tx_pin = sync;
                Tx_pin = stx;
                Tx_pin = mensagem;
                Tx_pin = end;
            }
            k_mutex_unlock(&mutex_ninguem_esta_enviando);
        }
        k_mutex_unlock(&mutex_envio);   
    }
}

void RX_proprio (void) {
    
    if((k_mutex_lock(&mutex_envio, K_FOREVER)) == 0){
        int recebi_TX = RX_pin;
        // comparar recebi com sync (fixo)
        // comparar recebi com stx (fixo)
        // comparar recebi com struct (ID + N + CHAR)
        // comparar recebi com end
        // se der errado, transmitir de novo (invocar a função de novo depois de um tempo aleatório que vai aumentando) (envio_correto = 0)
        // se der certo, envio_correto = 1;
    }
    k_mutex_unlock(&mutex_envio);
}

void RX_externo (void) {
    int recebi_externo = RX_pin

    // se eu parar de receber lixo (ou seja receber o caractere U), k_mutex_lock(&mutex_envio, K_FOREVER) // senão, return 0
    // comparar recebi_externo com sync
    // se der certo, comparar id com id_proprio
    // se o id for o seu, ler os caracteres e colocar na FIFO_RX
    // ignorar end
}

K_THREAD_DEFINE(TX_id, MY_STACK_SIZE, TX, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(RX_proprio_id, MY_STACK_SIZE, RX_proprio, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(RX_externo_id, MY_STACK_SIZE, RX_externo, NULL, NULL, NULL, MY_PRIORITY, 0, 0);]