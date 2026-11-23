"""
Planejamento de Trajetória com o algoritmo A*
Disciplina: Introdução a Robótica - UFMA / Ciência da Computação

- Ambiente: grid 15x15 
"""

import heapq
import math
import sys

# AMBIENTE (mapa  15 x 15)

MAPA_TEXTO = [
    "...............",  # 0
    "...............",  # 1
    ".......######..",  # 2
    "..##...######..",  # 3
    "..##...######..",  # 4
    "..##...........",  # 5
    "..##...........",  # 6
    "..##..####.....",  # 7
    "..##..####.....",  # 8
    "...............",  # 9
    "........##.....",  # 10
    "........##.....",  # 11
    "........##.....",  # 12
    "........##.....",  # 13
    "........##.....",  # 14
]


GRID = [[1 if c == "#" else 0 for c in linha] for linha in MAPA_TEXTO]
LINHAS = len(GRID)
COLUNAS = len(GRID[0])



# Algoritmo A*

def heuristica(a, b, diagonal):
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    if diagonal:
        return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)
    
    return dr + dc # Distância de Manhattan


def livre(pos):
    r, c = pos
    return 0 <= r < LINHAS and 0 <= c < COLUNAS and GRID[r][c] == 0


def vizinhos(pos, diagonal):
    r, c = pos
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        n = (r + dr, c + dc)
        if livre(n):
            yield n, 1.0
    if diagonal:
        for dr, dc in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            n = (r + dr, c + dc)
            if livre(n) and livre((r + dr, c)) and livre((r, c + dc)):
                yield n, math.sqrt(2)


def a_estrela(origem, destino, diagonal=False):
    contador = 0  
    aberta = [(heuristica(origem, destino, diagonal), contador, origem)]
    g = {origem: 0.0}
    pai = {origem: None}
    fechada = set()

    while aberta:
        _, _, atual = heapq.heappop(aberta)
        if atual in fechada:
            continue
        if atual == destino:
            caminho = []
            while atual is not None:
                caminho.append(atual)
                atual = pai[atual]
            caminho.reverse()
            return caminho
        fechada.add(atual)

        for viz, custo in vizinhos(atual, diagonal):
            if viz in fechada:
                continue
            novo_g = g[atual] + custo
            if viz not in g or novo_g < g[viz]:
                g[viz] = novo_g
                pai[viz] = atual
                contador += 1
                f = novo_g + heuristica(viz, destino, diagonal)
                heapq.heappush(aberta, (f, contador, viz))
    return None


def custo_caminho(caminho):
    total = 0.0
    for a, b in zip(caminho, caminho[1:]):
        total += math.sqrt(2) if (a[0] != b[0] and a[1] != b[1]) else 1.0
    return total


# saida
def desenhar(caminho=None, origem=None, destino=None):
    no_caminho = set(caminho) if caminho else set()
    print("     " + "".join("%2d " % c for c in range(COLUNAS)))
    print("    +" + "---" * COLUNAS + "+")
    for r in range(LINHAS):
        linha = ""
        for c in range(COLUNAS):
            p = (r, c)
            if p == origem:
                s = " S "
            elif p == destino:
                s = " G "
            elif GRID[r][c] == 1:
                s = "███"
            elif p in no_caminho:
                s = " * "
            else:
                s = " . "
            linha += s
        print(" %2d |%s|" % (r, linha))
    print("    +" + "---" * COLUNAS + "+")
    print("  S = origem   G = destino   * = trajetória   ███ = obstáculo\n")


def planejar_e_mostrar(origem, destino, diagonal=False):
    print("Origem: %s   Destino: %s   (movimentos: %s)"
          % (origem, destino, "8 direções" if diagonal else "4 direções"))
    caminho = a_estrela(origem, destino, diagonal)
    if caminho is None:
        print("Nenhum caminho encontrado entre origem e destino.\n")
        desenhar(None, origem, destino)
        return
    print("Posições (linha, coluna) a percorrer:")
    print(" -> ".join("(%d,%d)" % p for p in caminho))
    print("Passos: %d   Custo total: %.2f\n" % (len(caminho) - 1,
                                                custo_caminho(caminho)))
    desenhar(caminho, origem, destino)



#saida
def ler_posicao(rotulo):
    """Lê 'linha coluna' e valida se é uma célula livre. 'q' sai."""
    while True:
        txt = input("%s (linha coluna, ou 'q' para sair): " % rotulo).strip()
        if txt.lower() in ("q", "sair"):
            return None
        partes = txt.replace(",", " ").split()
        if len(partes) != 2 or not all(p.lstrip("-").isdigit() for p in partes):
            print("  Entrada inválida. Exemplo: 0 0")
            continue
        pos = (int(partes[0]), int(partes[1]))
        if not (0 <= pos[0] < LINHAS and 0 <= pos[1] < COLUNAS):
            print("  Fora do mapa. Use linha e coluna entre 0 e %d." % (LINHAS - 1))
            continue
        if GRID[pos[0]][pos[1]] == 1:
            print("  Essa célula é um obstáculo. Escolha uma célula livre.")
            continue
        return pos


def modo_interativo(diagonal):
    print("=== Planejamento de trajetória com A* ===\n")
    desenhar()
    while True:
        origem = ler_posicao("Origem ")
        if origem is None:
            break
        destino = ler_posicao("Destino")
        if destino is None:
            break
        print()
        planejar_e_mostrar(origem, destino, diagonal)


def modo_demo(diagonal):
    testes = [
        ((0, 0), (14, 14)),
        ((14, 0), (0, 14)),
        ((5, 0), (5, 14)),
        ((4, 4), (13, 7)),
        ((9, 0), (14, 10)),
        ((6, 5), (1, 9)),
        ((0, 0), (0, 0)),
    ]
    for i, (o, d) in enumerate(testes, 1):
        print("######## TESTE %d ########" % i)
        planejar_e_mostrar(o, d, diagonal)


if __name__ == "__main__":
    diag = "--diag" in sys.argv
    if "--demo" in sys.argv:
        modo_demo(diag)
    else:
        modo_interativo(diag)