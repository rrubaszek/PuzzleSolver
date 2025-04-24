import random

def count_inversions(tiles: list[int]) -> int:
    inv = 0
    tile_list = [t for t in tiles if t != 0]
    for i in range(len(tile_list)):
        for j in range(i + 1, len(tile_list)):
            if tile_list[i] > tile_list[j]:
                inv += 1
    return inv

def is_solvable(board: list[int], n: int) -> bool:
    inversions = count_inversions(board)
    
    if n % 2 == 1:
        return inversions % 2 == 0
    else:
        row_from_bottom = n - (board.index(0) // n)
        if row_from_bottom % 2 == 0:
            return inversions % 2 == 1
        else:
            return inversions % 2 == 0

def generate_board(n: int) -> list[int]:
    tiles = list(range(n * n))
    while True:
        random.shuffle(tiles)
        if is_solvable(tiles, n):
            return tiles
        
if __name__ == "__main__":
    n = int(input())
    print(generate_board(n))