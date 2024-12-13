#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

K_MUTEX_DEFINE(mutex_TX_RX);
K_MUTEX_DEFINE(mutex_RX_Buffer);

K_CONDVAR_DEFINE(cond_RX_TX);
K_CONDVAR_DEFINE(cond_RX_Buffer);

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 0

int Buffer = 0;
int recebido_RX = 0;
int reenviar = 0; // se algum byte enviado não for o esperado, reenviar a mensagem. Temos que implementar no codigo do TX uma forma de ele, toda vez que terminar de enviar o pacote, voltar essa variável para 0;

struct package {
    char U;
    char sync;
    char stx;
    char id;
    char letras[8];
    char end;
	char n;
};

struct package pacote_recebido;
struct package pacote_enviado;

void RX_Buffer (void){
    //k_msleep(10); // aguarda um tempo para iniciar
    int id = 0;
    int i = 0;
    int j = 0;
    int n = 0;
    int c = 0;

    while (1) {
    k_mutex_lock(&mutex_RX_Buffer, K_FOREVER);

    int aux = recebido_RX & 0b0110;

    if(aux == 0b0110) { // se os 2 do meio forem 1, colocar 1 no Buffer
        Buffer = (Buffer << 1) | 0b1;
    }
    if(aux == 0b0000) { // se os 2 do meio forem 0, colocar 0 no Buffer
        Buffer = (Buffer << 1) | 0b0;
    }

    i++;

    if (i % 8 == 0){ // se tiver chego ao último bit
        int posicao_byte = i / 8;

        if (posicao_byte == 1) {
            if (Buffer == pacote_enviado.U) { // se não for U, ele está lendo lixo (implementar mutex aq depois)
            }	
            else{
                i = 0;
            }
        }
        else if (posicao_byte == 2) {
            if (Buffer == pacote_enviado.sync) { // se não for sync, mensagem inválida. Recomeçar
                pacote_recebido.sync = Buffer;
            }
            else{
                i = 0;
            }
        }
        // se for o sync, armazenar todo o resto. Compararemos no final 
        else if (posicao_byte == 3) {
            pacote_recebido.stx = Buffer;
    
        }
        else if (posicao_byte == 4) {
            id = (Buffer & 0b00011111); // guarda os 5 primeiros bits para obter o id
            n = Buffer;
            n = ((n >> 5) & 0b00000111); // shifta 5 para direita para obter os 3 ultimos bits, o n// se for o mesmo id que você, sinalizar que não é necessário printar na tela. Caso contrário apenas armazenar e pedir para reenviar
            pacote_recebido.id = id;
            pacote_recebido.n = n;
            // aqui iremos tratar que qualquer id e n são válidos. O tratamento de printar ou não na tela será feito posteriormente
        }
        else if (posicao_byte >= 5 && posicao_byte < 5 + n) { // guarda o valor n de caracteres
            pacote_recebido.letras[c++] = Buffer;
        }
        else if (posicao_byte == 5 + n) {
            c = 0;
            pacote_recebido.end = Buffer;
            
                    if (pacote_recebido.stx == pacote_enviado.stx &&
                        pacote_recebido.n == pacote_enviado.n &&
                        pacote_recebido.id == pacote_enviado.id &&
                        memcmp(pacote_recebido.letras, pacote_enviado.letras, sizeof(pacote_recebido.letras)) == 0 &&
                        pacote_recebido.end == pacote_enviado.end) {
                    reenviar = 0;
                } 
                else {
                    reenviar = 1;
                }
            }
        }
    }

    k_condvar_signal(&cond_RX_Buffer); // libera novamente o RX para receber
    k_mutex_unlock(&mutex_RX_Buffer); // mutex para a variável condicional
}

K_THREAD_DEFINE(RX_Buffer_ID, MY_STACK_SIZE, RX_Buffer, NULL, NULL, NULL, MY_PRIORITY, 0, 10);