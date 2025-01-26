# Function to draw a detailed block with more color customization
def draw_colored_block(ax, xy, width, height, title, subblocks=None, color="lightblue", subcolor="lightyellow", title_color="black", fontsize=10):
    # Main block
    block = FancyBboxPatch(
        xy, width, height, boxstyle="round,pad=0.3", edgecolor="black", facecolor=color, lw=1.5
    )
    ax.add_patch(block)
    # Title of the block
    ax.text(
        xy[0] + width / 2,
        xy[1] + height - 0.2,
        title,
        ha="center",
        va="top",
        fontsize=fontsize,
        weight="bold",
        color=title_color
    )
    # Subblocks
    if subblocks:
        for i, subblock in enumerate(subblocks):
            ax.text(
                xy[0] + width / 2,
                xy[1] + height - 0.6 - i * 0.4,
                subblock,
                ha="center",
                va="center",
                fontsize=fontsize - 2,
                wrap=True,
                bbox=dict(facecolor=subcolor, edgecolor="black", boxstyle="round,pad=0.1"),
            )

# Create the figure and axes
fig, ax = plt.subplots(figsize=(10, 14))
ax.set_xlim(0, 12)
ax.set_ylim(0, 16)
ax.axis("off")

# Draw the architecture layers with more separation and colors
draw_colored_block(ax, (4, 14), 4, 1.5, "Input Layer", ["Sensor Data (Gyroscope, GPS, etc.)"], color="lightcoral", title_color="white")
draw_colored_block(ax, (4, 12), 4, 1.5, "Input Embedding Layer", ["Convert Sensor Data to Tokens"], color="lightblue")
draw_colored_block(ax, (4, 10), 4, 1.5, "Positional Encoding Layer", ["Add Temporal Information"], color="lightgreen")
draw_colored_block(ax, (4, 6), 4, 3.5, "Transformer Blocks", [
    "Layer Norm",
    "Multi-head Self-Attention",
    "Feedforward Network",
    "Residual Connections"
], color="lightpink", subcolor="mistyrose")
draw_colored_block(ax, (4, 4), 4, 1.5, "Dropout Layer", ["Regularization"], color="gold")
draw_colored_block(ax, (4, 2), 4, 1.5, "Fully Connected Layer", ["Predict Position, Velocity, Orientation"], color="lightgray")
draw_colored_block(ax, (4, 0), 4, 1.5, "Output Layer", ["Next Timestamp Predictions"], color="lightseagreen", title_color="white")

# Add arrows connecting the layers with more spacing
arrow_props = dict(facecolor="black", arrowstyle="->", lw=1.5)
ax.annotate("", xy=(6, 13.6), xytext=(6, 15.4), arrowprops=arrow_props)
ax.annotate("", xy=(6, 11.6), xytext=(6, 13.4), arrowprops=arrow_props)
ax.annotate("", xy=(6, 9.4), xytext=(6, 11.4), arrowprops=arrow_props)
ax.annotate("", xy=(6, 5.6), xytext=(6, 7.6), arrowprops=arrow_props)
ax.annotate("", xy=(6, 3.6), xytext=(6, 5.4), arrowprops=arrow_props)
ax.annotate("", xy=(6, 1.6), xytext=(6, 3.4), arrowprops=arrow_props)



# Add a more detailed Transformer Block breakdown on the side
draw_colored_block(ax, (8.5, 6), 3, 3.5, "Transformer Block Details", [
    "Layer Norm",
    "Multi-head Attention (Heads)",
    "Feedforward Network",
    "Residual Connections"
], color="lightpink", subcolor="mistyrose")

# Title
plt.title("Enhanced Architecture Diagram for 3D AI-Based Projectile Model", fontsize=14, weight="bold")

# Show the plot
plt.show()
