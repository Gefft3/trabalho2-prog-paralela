dataset="../datasets/ca_astroph.edgelist"
k_cliques=7
n_threads=16
carga_roubada=100

./programa $dataset $k_cliques $n_threads $carga_roubada

k_cliques=6
dataset="../datasets/dblp.edgelist"
./programa $dataset $k_cliques $n_threads $carga_roubada

bash ../versao1/call.sh