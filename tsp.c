/*
    Andrea Alexandra Sánez Oquendo RA:2126524
    Fernanda Rocha Costa Neto RA:2126605
    Patricia Abe Turato RA:2136007
*/
#include<stdio.h>
#include<stdlib.h>
#include<ctype.h>
#include<string.h>
#include<time.h>
#include<math.h>
#include <limits.h>
#include <stdbool.h>
#include <float.h>

#define MAX_XAXIS 1024
#define MAX_YAXIS 1024

/* A struct Coordenada é utilizada para colocar as posições x e y do ponto. */
typedef struct coordenada{
    int x; 
    int y;
} Coordenada;

/* A struct Vértice é utilizada para ver os vértices do grafo. O peso muda conforme a adição na lista de adjacências. */
typedef struct vertice {
  int id;
  struct vertice* prox;
  float peso; 
} Vertice;

/* A struct Grafo é a lista de Adjacências do Grafo. Coordenada* pos é o vetor de posições dos pontos e int* visitado é utilizado para fazer a Busca em Profundidade. */
typedef struct grafo {
  int V;
  int E;
  int* visitado; 
  Coordenada* pos; /* Vetor com as posições de cada vértice do grafo */
  Vertice** lista; /* Lista de adjacências */
} Grafo;

/*--- Funções do Grafo ---*/
Grafo* criar_grafo(int tam);
void destroi_grafo(Grafo* G);
void adiciona_aresta(Grafo* G, int u, int v);
void remove_aresta(Grafo* G, int u, int v);
float distancia_euclidiana(Coordenada a, Coordenada b);

/*--- Heap ---*/
void min_heapify(float custo[], int vertice[], int size, int i);
int heap_extract_min(float v[], int vertice[], int size);
void swap(int x, int y, int v[]);
int esquerda(int i);
int pai(int i);
int direita(int i);
void atualiza(float custo[], int vertice[], int size, int v);
int presente_vetor(int v[], int size, int el);

/*--- Funções Gerais do Trabalho ---*/
Grafo* Prim(Grafo* G);
void BuscaProfundidade(Grafo* G, int V, int* ordem, int *i);
void cria_arquivo_AGM(Grafo* G);
void cria_arquivo_ciclo(Grafo* G, int* v);
float valor_ciclo(Grafo* G, int* v);

/* 
** Retorna o pai do elemento no heap.
** Entrada: índice no vetor do heap.
** Saída: índice do pai do elemento no heap.
*/
int pai(int i){
	return (i-1)/2;
}

/* 
** Retorna o filho da esquerda do elemento no heap. 
** Entrada: i - índice no vetor do heap.
** Saída: índice do filho da esquerda no heap
*/
int esquerda(int i){
	return i*2+1;
}

/* 
** Retorna o filho da esquerda do elemento no heap. 
** Entrada: i - índice no vetor do heap.
** Saída: índice do filho da esquerda no heap
*/
int direita(int i){
	return i*2 +2;
}

/* 
** Troca dois elementos num vetor nos índices x e y. É utilizado para trocar os elementos. 
** Entrada: x - primeiro índice  
**          y - segundo índice
**          v - vetor a ser alterado
** Saída: sem retorno
*/
void swap(int x, int y, int v[]){
    int aux = v[x];
    v[x] = v[y];
    v[y] = aux;
}

/*
** Min Heapify. É utilizado o custo de cada vértice como parâmetro de comparação. 
** Entrada: vetor de custo, vetor vertice utilizado para o heap, tamanho do vetor vertice, elemento a ser verificado
** Saída: sem retorno
*/
void min_heapify(float custo[], int vertice[], int size, int i){
    int e = esquerda(i);
    int d = direita(i);
    int menor = i;
   
    if(e < size && custo[vertice[e]] < custo[vertice[menor]])
        menor = e;

    if(d < size && custo[vertice[d]] < custo[vertice[menor]])
        menor = d;

    if(menor!=i){
        swap(menor, i, vertice);
        min_heapify(custo, vertice, size, menor);
    }
}

/*
** Atualiza é a função Min Decrease Key exceto que, como é enviado só o id do vértice, procuramos onde ele está localizado no Heap. 
** Entrada: vetor de custo, vetor vertice utilizado para o heap, tamanho do vetor vertice, índice do vertice no qual o custo foi mudado
** Saída: sem retorno
*/
void atualiza(float custo[], int vertice[], int size, int v){
    
    int i;
    for(i = 0; i < size; i++){
        if(vertice[i] == v)
            break;
    }
    
    while(i<size && custo[vertice[pai(i)]]> custo[vertice[i]]){
        swap(pai(i), i, vertice);
        i = pai(i);
    }

}

/*
** Extrai o índice do vetor com menor custo no grafo fora do corte do Prim. 
** Entrada: vetor de custo, vetor vertice utilizado para o heap, tamanho do vetor vertice
** Saída: retorna o índice do vértice de menor custo
*/
int heap_extract_min(float custo[], int vertice[], int size){
    if(size < 1){
        printf("ERROR");
        return 0;
    }
    int min = vertice[0];

    vertice[0] = vertice[size - 1];

    min_heapify(custo, vertice, size - 1, 0);
    return min;
}

/*
** Aloca espaço para o grafo. 
** Entrada: tam - número máximo de vértices que o grafo pode ter
** Saída: Grafo vazio
*/
Grafo* criar_grafo(int tam){
    int v;
    Grafo *G = (Grafo *)malloc(sizeof(Grafo));

    G->V = tam;
    G->E = 0;
    G->pos = (Coordenada*)malloc(tam * sizeof(Coordenada));
    G->visitado = (int*)malloc(tam * sizeof(int));
    G->lista= (Vertice **)malloc(tam * sizeof(Vertice *));

    /* Número os vértice de 1 até Tamanho */
    for (v = 0; v < G->V; v++) {
       G->lista[v] = NULL;
       G->visitado[v] = 0;
    } 
    return G;
}

/* 
** Desaloca todos os elementos do grafo. 
** Entrada: G - Grafo a ser desalocado.
** Saída: sem retorno
*/
void destroi_grafo(Grafo* G){
   
    int v;
    for (v = 0; v < G->V; v++) {
       if (G->lista[v] != NULL) {
          free(G->lista[v]);
       }
    }
    free(G->pos);
    free(G->visitado);
    free(G->lista);
    free(G);
}

/* 
** Cria uma aresta entre os vértice u e v. 
** Entrada: G - Grafo G onde será adicionada a aresta e os 
            u, v - índices u e v dos vértices a serem conectados
** Saída: sem retorno
** Considerações: Cria um elemento de vértice, adiciona seu id e seu peso (distância euclidiana) em relação ao vértice ao qual está sendo conectado.
*/
void adiciona_aresta(Grafo *G, int u, int v){

    Vertice *temp, *ultimo = NULL;
    for (temp = G->lista[u]; temp != NULL; temp = temp->prox) {
       if (temp->id == v) {
          return;
       }
       ultimo = temp;
    }
    Vertice *novo = (Vertice *)malloc(sizeof(Vertice));
    novo->id= v;
    novo->prox = NULL;
    novo->peso = distancia_euclidiana(G->pos[u],G->pos[v]);
    if (ultimo != NULL) {
       ultimo->prox = novo; 
    }
    else { 
       G->lista[u] = novo; 
    }

    G->E++;
}

/* 
** Adiciona um vértice ao Grafo Completo. 
** Entrada: H - Grafo G a ao qual será adicionado o vértice
                v - índice do vértice
** Saída: sem retorno
** Considerações: Adiciona o vértice v à lista de todos os outros elementos do grafo exceto a dele mesmo.
*/
void adiciona_vertice(Grafo* G, int v){

    int i;
    for(i=0; i < G->V; i++){
        if(i!=v)
            adiciona_aresta(G, i, v);
    }
    
}

/* 
** Calcula a distância euclidiana entre duas coordenadas.
** Entrada: Coordenadas a e b.
** Saída: Distância Euclidiana entre as coordenadas a e b.
*/
float distancia_euclidiana(Coordenada a, Coordenada b){
    return sqrt(pow(a.x - b.x, 2)+pow(a.y - b.y, 2));
}

/* 
** É usado para Verificar se um determinado índice já se encontra no vetor de corte .
** Entrada: v - vetor v, 
            size - tamanho do vetor
            el - elemento a ser verificado se está no vetor.
** Saída: 1 se estiver no vetor, 0 se não estiver.
** Considerações: Percorre o vetor vendo se o elemento está lá.
*/
int presente_vetor(int v[], int size, int el){

    int i;
    for(i = 0; i < size; i++){
        if(v[i] == el)
            return 1;
    }
    return 0;
}

/* 
** Algoritmo de Prim
** Entrada: G - grafo criado na main a partir dos pontos do arquivo "input.txt"
** Saída: Retorna a árvore geradora mínima do grafo em questão 
** Considerações: Este algoritmo utiliza o heap para a escolha do próximo vértice a ser inserido no corte. Os vetores pai e custo armazenam, 
** respectivamente, o índice do vértice do pai e o custo do pai até o vértice em si. O vetor corte é usado para ver quais os vértices que estão no corte, e vertice é o heap.
** O grafo retornado é feito a partir do vetor pai.
*/
Grafo* Prim(Grafo* G){
    
    int V = G->V;
    
    Grafo* T = criar_grafo(V);
    
    /* Função de cada vetor:
        pai - armazena o índice do pai de cada vértice no índice respectivo ao vértice
        custo - mesma coisa que pai mas com o custo mínimo
        vertices - é nele que é feito o heap sort
    */
    int pai[V], vertices[V], corte[V];
    float custo[V];
    int size = V;
    
    int i;
    for (i = 0; i < V; i++) {
       vertices[i] = i;
       pai[i] = i;
       custo[i] = FLT_MAX;
    } 

    custo[0] = 0;
    atualiza(custo, vertices, size, 0);
    
    int v;
    i=0;
    while(size>0){
        v = heap_extract_min(custo, vertices, size);
        corte[i] = v;
        size--;
        i++;
        
        Vertice* temp = G->lista[v];
        while(temp != NULL){
            if(temp->peso <  custo[temp->id] && presente_vetor(corte, V - size - 1, temp->id) == 0){
                custo[temp->id] = temp->peso;
                pai[temp->id] = v;
                atualiza(custo, vertices, size, temp->id);
            }
            temp = temp->prox;
        }
    }

    /* Faz o grafo T baseado nos pais de cada elemento e reduz o número de arestas porque é não-direcionado.*/

    T->pos[0] = G->pos[0];
    for(i = 1; i < V; i++){
        adiciona_aresta(T, pai[i], i);
        adiciona_aresta(T, i, pai[i]);
        T->pos[i] = G->pos[i];
        T->E--;
    }

    return T;
}

/* 
** Algoritmo de Busca em Profundidade
** Realiza a busca em profundidade de um grafo a partir de um vértice dado
** Entradas: G - a árvore geradora mínima computada pelo algoritmo de Prim, 
**           V - o índice do primeiro vértice a ser visitado (mesmo vértice usado para iniciar o algoritmo de Prim),
**           ordem - vetor que vai armazenar a sequência em que os vértices serão visitados,
**           i - índice para o vetor ordem
** Saída: sem retorno  
** Considerações: o algortimo é recursivo, funcionando como uma pilha
*/
void BuscaProfundidade(Grafo* G, int V, int* ordem, int *i){
    Vertice* listaAdj = G->lista[V];

    G->visitado[V] = 1;
    ordem[*i] = V;

    while (listaAdj != NULL) {
        if (G->visitado[listaAdj->id] == 0) {
            *i = *i + 1;
            BuscaProfundidade(G, listaAdj->id, ordem, i);
        }
        listaAdj = listaAdj->prox;
    }
}

/* 
** Cria o aquivo tree.txt utilizando o grafo
** Entradas: G - a árvore geradora mínima computada pelo algoritmo de Prim,
**           v - vetor com a ordem do ciclo computado pela BuscaProfundidade
** Saída: não tem retorno, mas cria um arquivo chamado tree.txt com as posições x e y de todas as dupla vértices conectados por uma aresta.
*/
void cria_arquivo_AGM(Grafo* G){
    FILE *fp = fopen("tree.txt", "w");

    if (fp == NULL)
	{
		fprintf(stderr, "Falha ao criar tree.txt.\n");
		return ;
	}

    char pontos[MAX_YAXIS][MAX_XAXIS];
    int n = 0;
    int n2 = G->lista[n]->id;
    Vertice *v = G->lista[n];

    while (n < G->V)
	{
        if(!pontos[n][n2] && !pontos[n2][n]){

            pontos[n][n2] = 1;
            pontos[n2][n] = 1;

            fprintf(fp, "%d %d\n", G->pos[n].x, G->pos[n].y);
            fprintf(fp, "%d %d\n", G->pos[n2].x, G->pos[n2].y);
        } 
        if(v->prox != NULL){
            v = v->prox;
            n2 = v->id;
        }
        else{ 
            n++;
            if(n < G->V){
                v = G->lista[n];
                n2 = v->id;
            }
        }
	}

    fclose(fp);
}

/* 
** Cria o aquivo cycle.txt utilizando o vetor com a ordem dos vértices e o grafo 
** Entradas: G - a árvore geradora mínima computada pelo algoritmo de Prim,
**           v - vetor com a ordem do ciclo computado pela BuscaProfundidade
** Saída: não tem retorno, mas cria um arquivo chamado cycle.txt com as posições x e y dos vértices seguindo a ordem do ciclo.
*/
void cria_arquivo_ciclo(Grafo* G, int* v){
    FILE *fp = fopen("cycle.txt", "w");

    if (fp == NULL)
	{
		fprintf(stderr, "Falha ao criar cycle.txt.\n");
		return ;
	}

    int n = 0;

    while (n <= G->V)
	{
        fprintf(fp, "%d %d\n", G->pos[v[n]].x, G->pos[v[n]].y);
        n++;
	}

    fclose(fp);
}

/* 
** Faz o cálculo final do custo do ciclo utilizando o vetor com a ordem dos vértices do ciclo e o grafo
** Entradas: G - a árvore geradora mínima computada pelo algoritmo de Prim,
**           v - vetor com a ordem do ciclo computado pela BuscaProfundidade
** Saída: Retorna um valor float com a soma dos pesos de todas as arestas do ciclo.
*/
float valor_ciclo(Grafo* G, int* v){
    
    float valor = 0;

    int i;
    for(i = 0; i < G->V; i++){
        valor+= distancia_euclidiana(G->pos[v[i]], G->pos[v[i+1]]);
    }
    return valor;
}

int main(int argc, char* argv[]){

    if(argc !=2){
        return 1;
    }

    int tam, j, i=0;
    
    /* Define o tamanho do grafo */
    FILE* file = fopen ("input.txt", "r");
    fscanf (file, "%d", &tam);
    Grafo *G = criar_grafo (tam);

    /* Adiciona as arestas e os valores de posição no grafo */
    while(i < tam){
        fscanf(file, "%d %d", &G->pos[i].x, &G->pos[i].y);
        i++;
    }
    for(j = 0; j < G->V; j++){
        adiciona_vertice(G, j);
    }
    G->E = G->E/2; // Atualiza a quantidade de arestas 

    int v[tam+1];

    /* --- INÍCIO DO TEMPO DE EXECUÇÃO DO ALGORITMO --- */
    
    clock_t inicio = clock();  

    Grafo *T = Prim(G);
    
    i = 0;

    BuscaProfundidade(T, 0, v, &i); //Inicializa a busca em profundidade a partir do primeiro vértice, zero
    v[tam] = 0; // Adiciona o índice do primeiro vértice da sequência no final do vetor para fechar o ciclo
    float valor = valor_ciclo(G, v);

    clock_t fim = clock();
    
    /* --- FIM DO TEMPO DE EXECUÇÃO DO ALGORITMO --- */

    cria_arquivo_AGM(T);
    cria_arquivo_ciclo(T, v);

    printf("%.6f %.6f\n", 1.0*(fim-inicio)/CLOCKS_PER_SEC, valor);
    
    destroi_grafo(G);
    destroi_grafo(T);
    

    return 0;
}