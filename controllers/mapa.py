import json
import random
import os
import math

mapa_name="try"
# Caminho base para os arquivos
base_path = "C:\\Users\\guiva\\Documents\\Aulas\\3ANO\\ir\\cv-primitive-mapping\\worlds\\empty.wbt"
updated_map_path = f"C:\\Users\\guiva\\Documents\\Aulas\\3ANO\\ir\\cv-primitive-mapping\\worlds\\{mapa_name}.wbt"
world_size = 0.5  # Metade do tamanho do mundo para facilitar os cálculos

# Funções para criar as strings das formas
def create_rectangle_string(x, y, width, height, name):
    return f"""
DEF {name} Solid {{
  translation {x} {y} 0
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 1 1 1
        }}
      }}
      geometry Box {{
        size {width} {height} 0.05
      }}
      castShadows FALSE
      isPickable FALSE
    }}
  ]
  name "{name.lower()}"
}}
    """

def create_circle_string(x, y, radius, name):
    return f"""
DEF {name} Solid {{
  translation {x} {y} 0
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 1 0 0
        }}
      }}
      geometry Cylinder {{
        radius {radius}
        height 0.1
      }}
      castShadows FALSE
      isPickable FALSE
    }}
  ]
  name "{name.lower()}"
}}
    """

def create_triangle_string(x, y, base, height, name):
    # Ajusta a posição Z para alinhar a base do triângulo com o chão
    z_position = 0  # Isso centraliza o triângulo verticalmente ao redor de z=0, então subimos metade da altura
    depth=x
    return f"""
DEF {name} Solid {{
  translation {x} {y} {z_position}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 0 0 1
        }}
      }}
      geometry IndexedFaceSet {{
        coord Coordinate {{
          point [
            {-base / 2} 0 0,          # Base left vertex (front)
            {base / 2} 0 0,           # Base right vertex (front)
            0 {height} 0,             # Top vertex (front)
            {-base / 2} 0 {depth},    # Base left vertex (back)
            {base / 2} 0 {depth},     # Base right vertex (back)
            0 {height} {depth}        # Top vertex (back)
          ]
        }}
        coordIndex [
          0 1 2 -1,                  # Front face
          3 4 5 -1,                  # Back face
          0 1 4 3 -1,                # Left side face
          1 2 5 4 -1,                # Right side face
          2 0 3 5 -1                 # Bottom face
        ]
      }}
      castShadows FALSE
      isPickable FALSE
    }}
  ]
  name "{name.lower()}"
}}
"""

# Função para criar a string de um pentágono sólido com a base na parte superior e paredes sólidas
def create_pentagon_string(x, y, radius, height, name):
    return f"""
DEF {name} Solid {{
  translation {x} {y} {height / 2}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 0 4 0
        }}
      }}
      geometry IndexedFaceSet {{
        coord Coordinate {{
          point [
            0 {-radius} {height / 2},                        # Vértice 1 (topo)
            {radius * 0.9} {-radius * 0.3} {height / 2},    # Vértice 2 (topo)
            {radius * 0.5} {radius * 0.8} {height / 2},     # Vértice 3 (topo)
            {-radius * 0.5} {radius * 0.8} {height / 2},    # Vértice 4 (topo)
            {-radius * 0.91} {-radius * 0.3} {height / 2},  # Vértice 5 (topo)
            0 {-radius} {-height / 2},                      # Vértice 1 (baixo)
            {radius * 0.9} {-radius * 0.3} {-height / 2},   # Vértice 2 (baixo)
            {radius * 0.5} {radius * 0.8} {-height / 2},    # Vértice 3 (baixo)
            {-radius * 0.5} {radius * 0.8} {-height / 2},   # Vértice 4 (baixo)
            {-radius * 0.91} {-radius * 0.3} {-height / 2}  # Vértice 5 (baixo)
          ]
        }}
        coordIndex [
          0 1 2 3 4 -1,    # Face superior
          5 6 7 8 9 -1,    # Face inferior
          0 5 6 1 -1,      # Lado 1
          1 6 7 2 -1,      # Lado 2
          2 7 8 3 -1,      # Lado 3
          3 8 9 4 -1,      # Lado 4
          4 9 5 0 -1       # Lado 5
        ]
        solid TRUE
      }}
      castShadows TRUE
      isPickable FALSE
    }}
  ]
  name "{name.lower()}"
}}
"""

# Função para selecionar a forma e sua posição
def select_shape_and_position(shape_type, quadrant=None):
    if shape_type == 'rectangle':
        width = round(random.uniform(0.1, 0.3), 2)
        height = round(random.uniform(0.1, 0.3), 2)
        dimensions=(width,height)
    elif shape_type == 'circle':
        radius = round(random.uniform(0.05, 0.15), 2)
        dimensions = (radius,)
    elif shape_type == 'triangle':
        base = round(random.uniform(0.1, 0.3), 2)
        height = 0.1
        dimensions = (base, height)
    elif shape_type == 'pentagon':
        radius = round(random.uniform(0.05, 0.1), 2)
        height = 0.1
        dimensions = (radius, height)
    else:
        raise ValueError("Unsupported shape type")

    if quadrant:
        x_offset = (quadrant - 1) % 2 * world_size - world_size / 2
        y_offset = (quadrant - 1) // 2 * world_size - world_size / 2
        x = x_offset + random.uniform(-0.1, 0.1)
        y = y_offset + random.uniform(-0.1, 0.1)
    else:
        x = round(random.uniform(-0.3, world_size - 0.2), 2)
        y = round(random.uniform(-0.3, world_size - 0.2), 2)

    return shape_type, x, y, dimensions
annotations=[]
num_quadrants = 4 # Exemplo: O usuário quer figuras em 3 quadrantes

# Lê o conteúdo do mapa vazio apenas uma vez, antes do loop
with open(base_path, 'r') as file:
    map_content = file.read()

map_content_with_shapes = map_content.strip()  # Prepara o conteúdo inicial

# Processa cada quadrante escolhido
for quadrant in range(1, num_quadrants+1):
    shape_type = random.choice(['rectangle', 'circle', 'triangle', 'pentagon'])  # Escolhe uma forma aleatoriamente para cada quadrante
    shape_type, x, y, dimensions = select_shape_and_position(shape_type, quadrant)
    if shape_type == 'rectangle':
        shape_string = create_rectangle_string(x, y, dimensions[0], dimensions[1], "RECTANGLE")
        annotation = {'name': shape_type.upper(), 'type': 'rectangle', 'x': x, 'y': y, 'width': dimensions[0], 'height': dimensions[1]}
    elif shape_type == 'circle':
        shape_string = create_circle_string(x, y, dimensions[0], "CIRCLE")
        annotation = {'name': shape_type.upper(), 'type': 'circle', 'x': x, 'y': y, 'radius': dimensions[0]}
    elif shape_type == 'triangle':
        shape_string = create_triangle_string(x, y, dimensions[0], dimensions[1], "TRIANGLE")
        annotation = {'name': "TRIANGLE", 'type': 'triangle', 'x': x, 'y': y, 'base': dimensions[0], 'height': dimensions[1]}
    elif shape_type == 'pentagon':
        shape_string = create_pentagon_string(x, y, dimensions[0], dimensions[1], "PENTAGON")
        annotation = {'name': shape_type.upper(), 'type': 'pentagon', 'x': x, 'y': y, 'radius': dimensions[0], 'height': dimensions[1]}

    annotations.append(annotation)
    print(annotation)

    # Acumula cada forma no conteúdo do mapa
    map_content_with_shapes += "\n" + shape_string + "\n"

# Após o loop, salva todas as formas acumuladas de uma só vez no arquivo do mapa
with open(updated_map_path, 'w') as file:
    file.write(map_content_with_shapes)

# Salva as anotações em formato JSON
with open(f'{mapa_name}', 'w') as file:
    json.dump(annotations, file, indent=4)

print("Mapa atualizado com sucesso!")