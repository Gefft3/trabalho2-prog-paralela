dataset="../datasets/ca_astroph.edgelist"

k_cliques=4

n_threads=16

metodo="dynamic" #pode ser "static" "dynamic" ou "guided"
chunk=64 #mudar apenas se for usar o dynamic, senao nao faz diferenca

./programa $dataset $k_cliques $n_threads $metodo $chunk