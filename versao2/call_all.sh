dataset1="../datasets/ca_astroph.edgelist"

k_cliques=(3 4 5 6)
n_threads=16
carga_roubada=500

for k in "${k_cliques[@]}"; do
  ./programa $dataset1 $k $n_threads $carga_roubada
done


dataset2="../datasets/ca_astroph.edgelist"

for k in "${k_cliques[@]}"; do
  ./programa $dataset2 $k $n_threads $carga_roubada
done

dataset3="../datasets/dblp.edgelist"

for k in "${k_cliques[@]}"; do
  ./programa $dataset3 $k $n_threads $carga_roubada
done
