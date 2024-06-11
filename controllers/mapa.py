import json
import random
import os

# Caminho base para os arquivos
base_path = "C:\\Users\\guiva\\Documents\\Aulas\\3ANO\\ir\\cv-primitive-mapping\\worlds\\empty.wbt"

# Caminho para salvar o mapa atualizado
updated_map_path = "C:\\Users\\guiva\\Documents\\Aulas\\3ANO\\ir\\cv-primitive-mapping\\worlds\\updated_world.wbt"

# Tamanho do mundo (ajuste conforme necessário)
world_size = 0.5

# Função para criar a string de um retângulo
def create_rectangle_string(x, y, width, height, name):
    return f"""
DEF {name} Solid {{
  translation {x} {y} 0
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 0 0 1
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

# Função para criar a string de um círculo
def create_circle_string(x, y, radius, name):
    return f"""
DEF {name} Solid {{
  translation {x} {y} 0
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 0 0 1
        }}
      }}
      geometry Cylinder {{
        radius {radius}
        height 0.01
      }}
      castShadows FALSE
      isPickable FALSE
    }}
  ]
  name "{name.lower()}"
}}
    """

# Função para criar a string de um triângulo
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

# Lista de anotações para YOLO
annotations = []

# Escolhe aleatoriamente entre retângulo, círculo ou triângulo
shape_type = 'triangle'

# Escolhe aleatoriamente as dimensões e a posição da forma
x = round(random.uniform(0.1, world_size - 0.5), 2)
y = round(random.uniform(0.1, world_size - 0.5), 2)

if shape_type == 'rectangle':
    width = round(random.uniform(0.3, 0.6), 2)
    height = round(random.uniform(0.3, 0.6), 2)
    shape_string = create_rectangle_string(x, y, width, height, "RECTANGLE")
    annotation = {'name': "RECTANGLE", 'type': 'rectangle', 'x': x, 'y': y, 'width': width, 'height': height}
elif shape_type == 'circle':
    radius = round(random.uniform(0.15, 0.3), 2)
    shape_string = create_circle_string(x, y, radius, "CIRCLE")
    annotation = {'name': "CIRCLE", 'type': 'circle', 'x': x, 'y': y, 'radius': radius}
elif shape_type == 'triangle':
    base = round(random.uniform(0.1, 0.3), 2)
    height = 0.1
    shape_string = create_triangle_string(x, y, base, height, "TRIANGLE")
    annotation = {'name': "TRIANGLE", 'type': 'triangle', 'x': x, 'y': y, 'base': base, 'height': height}

annotations.append(annotation)
print(annotation)
# Lê o conteúdo do mapa vazio
with open(base_path, 'r') as file:
    map_content = file.read()

# Adiciona a forma ao conteúdo do mapa
map_content_with_shape = map_content.strip() + "\n" + shape_string + "\n"

# Salva o novo mapa com a forma adicionada
with open(updated_map_path, 'w') as file:
    file.write(map_content_with_shape)

# Salva as anotações em formato JSON
with open('annotations.json', 'w') as file:
    json.dump(annotations, file, indent=4)

print("Mapa atualizado com sucesso!")