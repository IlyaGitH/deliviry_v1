/* Shared metric workspace for the dashboard and camera diagnostics. */
const Grid100 = {
  bounds(grid = {}) {
    const [cx, cy] = grid.center || [0, 0];
    return {x0: cx - 50, x1: cx + 50, y0: cy - 50, y1: cy + 50, cx, cy};
  },
  view(grid, width, height, zoom = 1, pan = {x: 0, y: 0}) {
    const b = this.bounds(grid);
    const s = Math.max(1, Math.min(width, height) - 48) / 100 * zoom;
    return {s, ox: width / 2 - b.cx * s + pan.x * s,
      oy: height / 2 + b.cy * s + pan.y * s};
  },
  draw(ctx, grid, v) {
    const b = this.bounds(grid);
    const X = x => v.ox + x * v.s, Y = y => v.oy - y * v.s;
    ctx.save();
    ctx.fillStyle = '#101d24';
    ctx.fillRect(X(b.x0), Y(b.y1), 100 * v.s, 100 * v.s);
    ctx.font = '10px sans-serif';
    for (let i = 0; i <= 20; i++) {
      const x = b.x0 + i * 5, y = b.y0 + i * 5;
      ctx.strokeStyle = i % 2 ? '#1d303b' : '#314956';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(X(x), Y(b.y0)); ctx.lineTo(X(x), Y(b.y1)); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(X(b.x0), Y(y)); ctx.lineTo(X(b.x1), Y(y)); ctx.stroke();
      if (i % 2 === 0) {
        ctx.fillStyle = '#91a6b3';
        ctx.textAlign = 'center'; ctx.fillText(String(x), X(x), Y(b.y0) + 14);
        ctx.textAlign = 'right'; ctx.fillText(String(y), X(b.x0) - 5, Y(y) + 3);
      }
    }
    ctx.strokeStyle = '#6b8697'; ctx.lineWidth = 1.5;
    ctx.strokeRect(X(b.x0), Y(b.y1), 100 * v.s, 100 * v.s);
    ctx.fillStyle = '#bed1db'; ctx.textAlign = 'left';
    ctx.fillText('100 × 100 м · шаг 5 м', X(b.x0), Y(b.y1) - 8);
    ctx.restore();
  }
};
if (typeof module !== 'undefined') module.exports = Grid100;
