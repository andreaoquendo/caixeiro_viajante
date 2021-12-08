#A regra tsp tem como dependência o arquivo objeto tsp.o
#O comando cria um executável saida a partir do objeto tsp.o
tsp: tsp.o
	gcc tsp.o -o tsp -lm

#A regra tsp.o tem como dependência o arquivo fonte tsp.c
#O arquivo objeto tsp.o é criado a partir do arquivo fonte tsp.c
tsp.o: tsp.c
	gcc -c tsp.c

#A regra clean: apaga os arquivos .o ao digitarmos make clean no console
clean:
	rm -f *.o

#A regra execClean apaga o arquivo executável ao digitarmos make execClean no console
execClean:
	rm -f tsp