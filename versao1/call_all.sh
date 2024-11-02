k_cliques=6
n_threads=16
chunk=10 

dataset="../datasets/ca_astroph.edgelist"

schedule="static"

./programa $dataset $k_cliques $n_threads $schedule $chunk

schedule="dynamic"

./programa $dataset $k_cliques $n_threads $schedule $chunk

schedule="guided"

./programa $dataset $k_cliques $n_threads $schedule $chunk

# ---------------------------------------------------------#

dataset2="../datasets/dblp.edgelist"

schedule="static"

./programa $dataset2 $k_cliques $n_threads $schedule $chunk

schedule="dynamic"

./programa $dataset2 $k_cliques $n_threads $schedule $chunk

schedule="guided"

./programa $dataset2 $k_cliques $n_threads $schedule $chunk