import numpy as np
import matplotlib.pyplot as plt

def plot_data_from_file(file_name):
    # Ler os dados do arquivo
    data = np.loadtxt(file_name, delimiter=',')
    
    # Extrair as colunas
    col1 = data[24000:, 0]  # Filtrar dados a partir de x=23999
    col2 = data[24000:, 1]
    col3 = data[24000:, 2]
    
    # Criar subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    
    # Plotar cada coluna com grid e rótulos
    axs[0].plot(col1)
    axs[0].set_title('Evolução da posição em X')
    axs[0].set_xlabel('Número de amostras')
    axs[0].set_ylabel('Posição em X')
    axs[0].grid(True)
    
    axs[1].plot(col2)
    axs[1].set_title('Evolução da posição em Y')
    axs[1].set_xlabel('Número de amostras')
    axs[1].set_ylabel('Posição em Y')
    axs[1].grid(True)
    
    axs[2].plot(col3)
    axs[2].set_title('Evolução da posição em Z')
    axs[2].set_xlabel('Número de amostras')
    axs[2].set_ylabel('Posição em Z')
    axs[2].grid(True)
    
    # Ajustar layout
    plt.tight_layout()
    plt.show()

# Plotando os dados
plot_data_from_file('d_pos.txt')

