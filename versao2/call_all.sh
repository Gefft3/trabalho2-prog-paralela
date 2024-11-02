dataset1="../datasets/ca_astroph.edgelist"
dataset2="../datasets/dblp.edgelist"

k_cliques=(5 6)
n_threads=16
carga_roubada=500

for k in "${k_cliques[@]}"; do
  ./programa $dataset1 $k $n_threads $carga_roubada
done


for k in "${k_cliques[@]}"; do
  ./programa $dataset2 $k $n_threads $carga_roubada
done

