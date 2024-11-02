#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include <map>
#include <vector>
#include "sort.h"
#include <algorithm>
#include <set>
#include <mutex>
#include <queue>
#include <atomic>
#include <random>
#include <cstdlib>
#include <omp.h>

using namespace std;
using namespace std::chrono;

class Graph {
    public:

        vector<pair<int, int>> arestas;
        vector<set<int>> vizinhos; 
        set<int> vertices; 

        Graph(vector<pair<int, int>> edgeList) {
            for (auto edge : edgeList) {
                const int tamanho_vizinhos = vizinhos.size();
                if (tamanho_vizinhos <= max(edge.first, edge.second)) {
                    vizinhos.resize(max(edge.first, edge.second) + 1);
                }
                arestas.push_back(edge);
                vertices.insert(edge.first); 
                vertices.insert(edge.second); 
                vizinhos[edge.first].insert(edge.second); 
                vizinhos[edge.second].insert(edge.first); 
            }
        }

        int contagem_cliques_paralela_balanceada(long unsigned int k, int n_threads, long unsigned  roubo_carga);
        bool esta_na_clique(int vertex, vector<int> clique);
        bool se_conecta_a_todos_os_vertices_da_clique(int vertex, vector<int> clique);
        bool formar_clique(int vertex, vector<int> clique);

        vector<int> getNeighbours(int vertex) {
            vector<int> neighbours;
            for (int neighbour : vizinhos[vertex]) {
                neighbours.push_back(neighbour);
            }
            return neighbours;
        }

        bool isNeighbour(int vertex, int neighbour) {
            return vizinhos[vertex].find(neighbour) != vizinhos[vertex].end();
        }

        void printar_grafo() {
            for (auto v : vertices) {
                cout << v << ": ";
                for (auto n : vizinhos[v]) {
                    cout << n << " ";
                }
                cout << endl;
            }
        }

        void printar_clique(vector<int> clique) {
            for (auto v : clique) {
                cout << v << " ";
            }
        }

        void release() {
            arestas.clear();
            vertices.clear();
            vizinhos.clear();
        }
};


vector<pair<int, int>> rename(const string& dataset) {
    
    ifstream inputFile(dataset);
    map<int, int> nodeMap;
    vector<pair<int, int>> edges;
    int nodeCounter = 0;
    
    if (!inputFile.is_open()) {
        cerr << "nao abriu, arquivo: " << dataset << endl;
    }

    int u, v;

    while (inputFile >> u >> v) {
        if (nodeMap.find(u) == nodeMap.end()) {
            nodeMap[u] = nodeCounter++;
        }
        if (nodeMap.find(v) == nodeMap.end()) {
            nodeMap[v] = nodeCounter++;
        }
        edges.push_back({nodeMap[u], nodeMap[v]});
    }

    inputFile.close();

    return edges;
}

bool Graph::esta_na_clique(int vertex, vector<int> clique) {
    for (size_t i = 0; i < clique.size(); i++) {
        if (vertex == clique[i]) {
            return true;
        }
    }
    return false;
}

bool Graph::se_conecta_a_todos_os_vertices_da_clique(int vertex, vector<int> clique) {
    for (int v : clique) {
        if (!isNeighbour(vertex, v)) {
            return false;
        }
    }
    return true;
}

bool Graph::formar_clique(int vertex, vector<int> clique) {
    bool cond1 = se_conecta_a_todos_os_vertices_da_clique(vertex, clique);
    bool cond2 = esta_na_clique(vertex, clique);

    bool condf = (cond1 && !cond2); 

    return condf;
}

bool clique_ja_existe(const set<vector<int>>& cliques, const vector<int>& clique) {
    return cliques.find(clique) != cliques.end();
}

int Graph::contagem_cliques_paralela_balanceada(long unsigned int k, int n_threads, long unsigned int roubo_carga) {
    unsigned int num_threads = n_threads;

    if (num_threads == 0) {
        num_threads = 1; 
    }

    vector<vector<int>> cliques_iniciais;
    for (auto v : vertices) {
        cliques_iniciais.push_back({v});
    }

    vector<vector<vector<int>>> cliques_por_thread(num_threads);
    size_t num_cliques = cliques_iniciais.size();
    size_t cliques_por_thread_size = num_cliques / num_threads;
    size_t excesso = num_cliques % num_threads;

    size_t indice = 0;
    for (unsigned int tid = 0; tid < num_threads; ++tid) {
        size_t num_para_thread = cliques_por_thread_size + (tid < excesso ? 1 : 0);
        for (size_t j = 0; j < num_para_thread; ++j) {
            cliques_por_thread[tid].push_back(cliques_iniciais[indice++]);
        }
    }

    vector<int> contagens(num_threads, 0);

    vector<omp_lock_t> locks(num_threads);
    for (unsigned int i = 0; i < num_threads; ++i) {
        omp_init_lock(&locks[i]);
    }

    omp_set_num_threads(num_threads);

    atomic<bool> work_done(false);

    #pragma omp parallel
    {
        unsigned int tid = omp_get_thread_num();
        int local_count = 0;

        srand(time(NULL) + tid);

        while (true) {
            vector<int> clique;
            bool has_work = false;

            omp_set_lock(&locks[tid]);
            if (!cliques_por_thread[tid].empty()) {
                clique = cliques_por_thread[tid].back();
                cliques_por_thread[tid].pop_back();
                has_work = true;
            }
            omp_unset_lock(&locks[tid]);

            if (has_work) {

                long unsigned tamanho_clique = clique.size();
                if (tamanho_clique == k) {
                    local_count++;
                    continue;
                }

                int ultimo_vertice = clique.back();

                vector<vector<int>> novas_cliques;

                vector<int> vizinhos_ultimo = getNeighbours(ultimo_vertice);

                for (int vizinho : vizinhos_ultimo) {
                    if (vizinho > ultimo_vertice && se_conecta_a_todos_os_vertices_da_clique(vizinho, clique)) {
                        vector<int> nova_clique = clique;
                        nova_clique.push_back(vizinho);
                        novas_cliques.push_back(nova_clique);
                    }
                }

                omp_set_lock(&locks[tid]);
                cliques_por_thread[tid].insert(cliques_por_thread[tid].end(), novas_cliques.begin(), novas_cliques.end());
                omp_unset_lock(&locks[tid]);

            } else {

                bool roubou = false;

                for (unsigned int attempt = 0; attempt < num_threads; ++attempt) {
                    unsigned int victim_tid = rand() % num_threads;

                    if (victim_tid == tid) continue;

                    if (omp_test_lock(&locks[victim_tid])) {
                        if (!cliques_por_thread[victim_tid].empty()) {
                    
                            size_t steal_count = min(roubo_carga, cliques_por_thread[victim_tid].size());

                            vector<vector<int>> stolen_cliques(
                                cliques_por_thread[victim_tid].end() - steal_count,
                                cliques_por_thread[victim_tid].end());

                            cliques_por_thread[victim_tid].erase(
                                cliques_por_thread[victim_tid].end() - steal_count,
                                cliques_por_thread[victim_tid].end());

                            omp_unset_lock(&locks[victim_tid]);

                            omp_set_lock(&locks[tid]);
                            cliques_por_thread[tid].insert(
                                cliques_por_thread[tid].end(),
                                stolen_cliques.begin(),
                                stolen_cliques.end());
                            omp_unset_lock(&locks[tid]);

                            roubou = true;
                            break; 
                        }
                        omp_unset_lock(&locks[victim_tid]);
                    }
                }

                if (!roubou) {
                    bool all_done = true;
                    for (unsigned int i = 0; i < num_threads; ++i) {
                        if (!cliques_por_thread[i].empty()) {
                            all_done = false;
                            break;
                        }
                    }
                    if (all_done) {
                        work_done.store(true);
                        break;
                    }
                }
            }
        }

        contagens[tid] = local_count;
    }

    for (unsigned int i = 0; i < num_threads; ++i) {
        omp_destroy_lock(&locks[i]);
    }

    int total_count = 0;
    for (int c : contagens) {
        total_count += c;
    }

    return total_count;
}

int main(int argc, char* argv[]) {

    string dataset = argv[1];
    int k_cliques = atoi(argv[2]);
    int n_threads = atoi(argv[3]);
    int roubo_carga = atoi(argv[4]);    
    
    
    vector<pair<int, int>> edges = rename(dataset);
    Graph* g = new Graph(edges);

    auto start = high_resolution_clock::now();
    int result = g->contagem_cliques_paralela_balanceada(k_cliques, n_threads, roubo_carga);
    auto end = high_resolution_clock::now();
    duration<double> duration = end - start;

    cout << "Dataset: " << dataset << endl;
    cout << "Tamanho do k: " << k_cliques << endl;
    cout << "Resultado: " << result << endl;
    cout << "Tempo de execução: " << duration.count() << " segundos" << endl;
    g->release();
    delete g;
}