dataset="../datasets/ca_astroph.edgelist"

k_cliques=5

n_threads=20

metodo="guided" #pode ser "static" "dynamic" ou "guided"
chunk=10 #mudar apenas se for usar o dynamic, senao nao faz diferenca

./programa $dataset $k_cliques $n_threads $metodo $chunk