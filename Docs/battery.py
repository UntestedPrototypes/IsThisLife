from designVals import *
from Pendulum_Calculator import *
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt


''' Estimation on power requirements and respective capacity required'''



def battery_sizing(I_nom, V_nom, max_discharge_sys, max_discharge_batt, battV, battC, run_time_min):
    '''Estimate cells in series and parallel per battery pack and total number of packs.'''
    # Cells 
    Nseries = np.ceil(V_nom / battV)
    Nparallel = 1

    cellsperpack = Nseries * Nparallel
    # Packs
    reqC = I_nom*run_time_min/60/0.8  # [Ah]
    
    option1 = np.ceil(reqC / (Nparallel * battC))
    option2 = np.ceil(max_discharge_sys / (max_discharge_batt ))*Nseries
    total_packs = np.maximum(option1, option2)

    total_cells = total_packs * Nparallel * Nseries
    return cellsperpack, total_packs, reqC, total_cells



def battery_comparison(pd_battery, V_nom, I_nom, run_time_min):
    """Compare different battery types based on total cells required and plot."""
    # Use the correct name column
    name_col = "battery_name" if "battery_name" in pd_battery.columns else "name"


    

    # Create two subplots: one for Total Packs vs Total Cells and one for Cells per Pack vs Total Cells
    fig, axs = plt.subplots(1, 3, figsize=(18, 6))

    # Reset legend handles for each subplot by re-iterating through the battery data
    for _, row in pd_battery.iterrows():
        cellsperpack, total_packs, total_Ah, total_cells = battery_sizing(
            I_nom,
            V_nom,
            80,
            row["max_continuous_discharge_A"],
            row["nominal_voltage_V"],
            row["capacity_Ah"],
            run_time_min,
        )

        label = row.get("battery_name", row.get("name", "Unnamed"))
        print(f"Name: {label}, Total Cells: {total_cells}")
        axs[0].scatter(total_cells, total_packs, marker="o", label=label)
        axs[1].scatter(total_cells, cellsperpack, marker="o", label=label)
        axs[2].scatter(total_packs, cellsperpack, marker="o", label=label)

    axs[0].set_xlabel("Total Cells")
    axs[0].set_ylabel("Total Packs")
    axs[0].set_title("Total Packs vs Total Cells")
    axs[0].legend(title="Battery")

    axs[1].set_xlabel("Total Cells")
    axs[1].set_ylabel("Cells per Pack")
    axs[1].set_title("Cells per Pack vs Total Cells")
    axs[1].legend(title="Battery")

    axs[2].set_xlabel("Total Packs")
    axs[2].set_ylabel("cellsperpack")
    axs[2].set_title("Total Cells vs cellsperpack")
    axs[2].legend(title="Battery")

    plt.tight_layout()
    plt.show()





if __name__ == "__main__":
    battery_comparison(pd_battery, V_nom, 2, desired_run_time)

    # I_nom = np.linspace(1, 10, 100)  # [Ah]

    # # Define a color palette using a colormap
    # colors = plt.get_cmap('tab10').colors

    # # For each battery in the dataframe, generate separate subplots in multiple columns.
    # n_batteries = pd_battery.shape[0]
    # n_cols = 2 if n_batteries > 1 else 1
    # n_rows = int(np.ceil(n_batteries / n_cols))
    
    # fig, axs = plt.subplots(n_rows, n_cols, figsize=(12 * n_cols, 5 * n_rows), constrained_layout=True)
    # # Flatten axs array for easier indexing, even if it's a single row or single subplot.
    # if n_batteries > 1:
    #     axs = np.array(axs).flatten()
    # else:
    #     axs = [axs]
    
    # for i, (_, row) in enumerate(pd_battery.iterrows()):
    #     cellsperpack, total_packs, total_Ah, total_cells = battery_sizing(
    #         I_nom,
    #         V_nom,
    #         80,  # system's max discharge (adjust as needed)
    #         row["max_continuous_discharge_A"],
    #         row["nominal_voltage_V"],
    #         row["capacity_Ah"],
    #         desired_run_time  # ensure desired_run_time is defined
    #     )

    #     battery_label = row.get("battery_name", row.get("name", "Unnamed"))
        
    #     ax = axs[i]
    #     # Choose a different color for each subplot
    #     color = colors[i % len(colors)]
    #     ax.plot(I_nom, total_cells, marker='o', color=color)
    #     ax.set_xlabel("Nominal Current (A)")
    #     ax.set_ylabel("Total Cells")
    #     ax.set_title(f"{battery_label}: Total Cells vs Nominal Current (Cells per Pack: {int(cellsperpack)})")
    
    # # Remove any unused subplots
    # for j in range(i + 1, len(axs)):
    #     fig.delaxes(axs[j])
        
    # plt.show()


    