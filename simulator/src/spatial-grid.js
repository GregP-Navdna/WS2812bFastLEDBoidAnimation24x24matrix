/**
 * SpatialGrid - Spatial partitioning for efficient neighbor lookup
 * Ported from spatial_grid.h
 */
export class SpatialGrid {
  constructor(gridWidth, gridHeight, worldWidth, worldHeight, maxObjectsPerCell = 6) {
    this.gridWidth = gridWidth;
    this.gridHeight = gridHeight;
    this.cellWidth = worldWidth / gridWidth;
    this.cellHeight = worldHeight / gridHeight;
    this.maxObjectsPerCell = maxObjectsPerCell;

    // Pre-allocate grid cells
    this.grid = new Array(gridWidth * gridHeight);
    for (let i = 0; i < this.grid.length; i++) {
      this.grid[i] = [];
    }

    // Reusable neighbor buffer to avoid allocations
    this._neighborBuffer = [];
  }

  clear() {
    for (let i = 0; i < this.grid.length; i++) {
      this.grid[i].length = 0;
    }
  }

  insert(object, x, y) {
    let cellX = Math.floor(x / this.cellWidth);
    let cellY = Math.floor(y / this.cellHeight);

    // Clamp to grid boundaries
    cellX = Math.max(0, Math.min(cellX, this.gridWidth - 1));
    cellY = Math.max(0, Math.min(cellY, this.gridHeight - 1));

    const cellIndex = cellY * this.gridWidth + cellX;
    if (this.grid[cellIndex].length < this.maxObjectsPerCell) {
      this.grid[cellIndex].push(object);
    }
  }

  getNeighbors(x, y, radius) {
    // Reuse buffer to avoid heap allocation
    this._neighborBuffer.length = 0;

    const minCellX = Math.max(0, Math.floor((x - radius) / this.cellWidth));
    const maxCellX = Math.min(this.gridWidth - 1, Math.floor((x + radius) / this.cellWidth));
    const minCellY = Math.max(0, Math.floor((y - radius) / this.cellHeight));
    const maxCellY = Math.min(this.gridHeight - 1, Math.floor((y + radius) / this.cellHeight));

    for (let cellY = minCellY; cellY <= maxCellY; cellY++) {
      for (let cellX = minCellX; cellX <= maxCellX; cellX++) {
        const cellIndex = cellY * this.gridWidth + cellX;
        const cell = this.grid[cellIndex];
        for (let i = 0; i < cell.length; i++) {
          this._neighborBuffer.push(cell[i]);
        }
      }
    }

    return this._neighborBuffer;
  }
}
