import numpy as np
import matplotlib.pyplot as plt

def plot_data_from_file(file_name):
    # Ler os dados do arquivo
    data = np.loadtxt(file_name, delimiter=',')
    
    # Extrair as colunas
    col1 = data[:, 0]
    col2 = data[:, 1]
    col3 = data[:, 2]
    
    # Criar subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    
    # Plotar cada coluna com grid, rótulos e linha vertical
    axs[0].plot(col1)
    axs[0].set_title('Evolução da aceleração em X')
    axs[0].set_xlabel('Número de amostras')
    axs[0].set_ylabel('Aceleração em X')
    axs[0].axvline(x=24000, color='r', linestyle='--')
    axs[0].grid(True)
    
    axs[1].plot(col2)
    axs[1].set_title('Evolução da aceleração em Y')
    axs[1].set_xlabel('Número de amostras')
    axs[1].set_ylabel('Aceleração em Y')
    axs[1].axvline(x=24000, color='r', linestyle='--')
    axs[1].grid(True)
    
    axs[2].plot(col3)
    axs[2].set_title('Evolução da aceleração em Z')
    axs[2].set_xlabel('Número de amostras')
    axs[2].set_ylabel('Aceleração em Z')
    axs[2].axvline(x=24000, color='r', linestyle='--')
    axs[2].grid(True)
    
    # Ajustar layout
    plt.tight_layout()
    plt.show()

# Plotando os dados
plot_data_from_file('d_acc.txt')

