# Blantyre-Shortest-Path-Assignment
Implementation of Dijkstra algorithm using Blantyre based weighted graph 
// Blantyre Road Network: Dijkstra's shortest path and Kruskal's MST//2026 DSA Assignment. Edge weights are road distances in km 
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <algorithm>
#include <limits>
#include <numeric>
#include <iomanip>
#include <stdexcept>
using namespace std;
const double INF = numeric_limits<double>::infinity();
struct Edge { int to; double w; };struct RawEdge { string a, b; double w; };struct IdEdge { int u, v; double w; };

const vector<string> NAMES = {    
"Limbe","Blantyre_Town","Soche","Chitawira","Nkolokosa","Nancholi","Naperi",    "Chichiri","Chilimba","Mbayani","Lunzu","Chileka","Kameza","Ndirande",    "Machinjiri","Nyambadwe","Namiwawa","Chilomoni","Bangwe","Zingwangwa"
};
const vector<RawEdge> RAW = {   
 {"Limbe","Bangwe",4.9},{"Limbe","Chitawira",5.0},{"Limbe","Chichiri",4.5},    {"Chichiri","Blantyre_Town",3.1},{"Blantyre_Town","Mbayani",3.2},{"Blantyre_Town","Namiwawa",2.4},    {"Mbayani","Chilimba",5.2},{"Chilimba","Kameza",3.3},{"Kameza","Lunzu",9.3},{"Kameza","Machinjiri",3.0},    {"Kameza","Chileka",4.7},{"Chitawira","Nkolokosa",2.0},{"Chitawira","Naperi",1.6},    {"Nkolokosa","Soche",1.9},{"Namiwawa","Chilomoni",3.5},{"Chichiri","Ndirande",5.5},    {"Ndirande","Blantyre_Town",4.0},{"Blantyre_Town","Nyambadwe",4.3},{"Nyambadwe","Ndirande",4.5},    {"Naperi","Nancholi",6.1},{"Naperi","Zingwangwa",1.6}}; 
 class Graph {public:    
  int n;  
  vector<vector<Edge>> adj;  
  vector<IdEdge> edges; 
   map<string,int> id; 
   Graph() : n(NAMES.size()), adj(NAMES.size()) {        for (int i = 0; i < n; i++) id[NAMES[i]] = i;  
for (const auto& e : RAW) {            
if (!id.count(e.a) || !id.count(e.b))                
throw runtime_error("Unknown place in RAW edge list: " + e.a + " / " + e.b);            
addEdge(id[e.a], id[e.b], e.w);        
}    
}    
void addEdge(int u, int v, double w) {        
adj[u].push_back({v, w});       
 adj[v].push_back({u, w});  
 // undirected        
edges.push_back({u, v, w}); 
   }    
int find(const string& s)
 const {        
auto it = id.find(s);  
      return it == id.end() ? -1 : it->second;    
}};
// Dijkstra with a min-priority queue,
 O((V+E) log V)void dijkstra(const Graph& g, int src, vector<double>& dist, vector<int>& prev, bool trace) {    
dist.assign(g.n, INF);   
 prev.assign(g.n, -1);    
vector<bool> done(g.n, false);  
  priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;    
dist[src] = 0;    
pq.push({0, src});   
 int step = 0;    
while (!pq.empty()) {        
double d = pq.top().first;        
int u = pq.top().second;     
   pq.pop();        
if (done[u]) continue;   
// stale queue entry        
done[u] = true;       
 if (trace) cout << "Step " << ++step << ": settle " << NAMES[u] << " (dist " << d << ")\n";        
for (const Edge& e : g.adj[u]) {            
if (!done[e.to] && d + e.w < dist[e.to]) {                dist[e.to] = d + e.w;                prev[e.to] = u;   
             pq.push({dist[e.to], e.to});                
if (trace) cout << "    relax " << NAMES[e.to] << " -> " << dist[e.to] << "\n";            }  
      }    
}}
string pathTo(const Graph& g, const vector<int>& prev, int dst) {    vector<int> p;    
for (int v = dst; v != -1; v = prev[v]) p.push_back(v);    reverse(p.begin(), p.end());    string s;    
for (size_t i = 0; i < p.size(); i++) s += (i ? " -> " : "") + NAMES[p[i]];    
return s;}
// Disjoint set union for Kruskalstruct 
DSU {    
vector<int> p;    
DSU(int n) : p(n) { iota(p.begin(), p.end(), 0); 
}    
int f(int x) { 
return p[x] == x ? x : p[x] = f(p[x]);
 }    
bool unite(int a, int b) { 
a = f(a); 
b = f(b); 
if (a == b) 
return false; 
p[a] = b; 
return true; 
}};
void kruskal(const Graph& g) {    
vector<IdEdge> es = g.edges;    
stable_sort(es.begin(), es.end(), [](const IdEdge& a, const IdEdge& b){ return a.w < b.w; });    
DSU d(g.n);   
 double total = 0; 
int count = 0; 
 cout << "Kruskal MST edges:\n";    for (const auto& e : es) {        
if (d.unite(e.u, e.v)) {            
cout << "  " << NAMES[e.u] << " -- " << NAMES[e.v] << " (" << e.w << ")\n";            
total += e.w; count++;       
 }    
}    
if (count == g.n - 1) 
cout << "MST total weight = " << total << " km, edges = " << count << "\n";    
else cout << "Graph is disconnected: spanning forest only (" << count << " edges, weight " << total << ")\n";
}
void showDijkstra(const Graph& g, const string& s, const string& t, bool trace) {    
int src = g.find(s);    
if (src < 0) { 
cout << "Error: unknown place '" << s << "'\n"; return;
 }    vector<double> dist; 
vector<int> prev;    
dijkstra(g, src, dist, prev, trace);    
if (t.empty()) {       cout << "Shortest distances from " << s << ":\n";        for (int i = 0; i < g.n; i++) {            
if (dist[i] == INF) cout << "  " << NAMES[i] << ": unreachable\n";            
else cout << "  " << NAMES[i] << ": " << dist[i] << " km  [" << pathTo(g, prev, i) << "]\n";       
 }        
return;    
}    
int dst = g.find(t);   
 if (dst < 0) {
 cout << "Error: unknown place '" << t << "'\n"; 
return; 
}    
if (dist[dst] == INF) 
cout << "No route from " << s << " to " << t << "\n";    else cout << s << " to " << t << ": " << dist[dst] << " km\nRoute: " << pathTo(g, prev, dst) << "\n";
}
int main(int argc, char* argv[]) {    
cout << fixed << setprecision(1);    
Graph g;    
if (argc >= 3 && string(argv[1]) == "dijkstra") {        
showDijkstra(g, argv[2], argc >= 4 ? argv[3] : "", false);        return 0;    
}    
if (argc >= 3 && string(argv[1]) == "trace") { showDijkstra(g, argv[2], "", true);
 return 0; 
}    
if (argc >= 2 && string(argv[1]) == "mst") { 
kruskal(g); return 0;
 }   
 // Interactive menu    
while (true) {        
cout << "\n=== Blantyre Road Network ===\n1. Shortest path (Dijkstra)\n2. All distances from a place\n"                "3. Minimum spanning tree (Kruskal)\n4. List places\n0. Exit\nChoice: ";        
int c;        
if (!(cin >> c)) break;        
if (c == 0) break;        
string a, b;        
if (c == 1) {
 cout << "From: "; 
cin >> a; 
cout << "To: "; 
cin >> b; 
showDijkstra(g, a, b, false);
 }        
else if (c == 2) {
 cout << "From: "; 
cin >> a; 
showDijkstra(g, a, "", false);
 }        
else if (c == 3) kruskal(g);        
else if (c == 4) { 
for (auto& s : NAMES) 
cout << s << "\n";
 }        
else cout << "Invalid choice.\n";   
 }   
 return 0;
}