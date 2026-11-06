/* Geometry-only editor: shared intersections are explicit, including splits of one-way roads. */
class RoadEditor {
  constructor(data) {
    this.data = JSON.parse(JSON.stringify(data || {}));
    this.data.nodes ||= {}; this.data.edges ||= []; this.data.directed_edges ||= [];
    this.selected = null; this.history = [];
  }
  snapshot() { this.history.push(JSON.stringify({data:this.data, selected:this.selected})); if (this.history.length>80) this.history.shift(); }
  undo() { const old=this.history.pop(); if(old) Object.assign(this,JSON.parse(old)); }
  nearestEdge(x,y,radius) {
    let best=null;
    for(const key of ['edges','directed_edges']) this.data[key].forEach(([a,b],i)=>{
      const A=this.data.nodes[a],B=this.data.nodes[b],dx=B[0]-A[0],dy=B[1]-A[1],length=dx*dx+dy*dy;
      if(!length)return;
      const t=Math.max(0,Math.min(1,((x-A[0])*dx+(y-A[1])*dy)/length));
      const p=[A[0]+t*dx,A[1]+t*dy],d=Math.hypot(x-p[0],y-p[1]);
      if(d<=radius && (!best || d<best.d)) best={key,i,a,b,p,d};
    });
    return best;
  }
  click(x,y,radius,remove=false) {
    const edge=this.nearestEdge(x,y,radius);
    if(remove) { if(edge){this.snapshot();this.data[edge.key].splice(edge.i,1);this.selected=null;} return; }
    this.snapshot();
    let id=null,dist=radius;
    Object.entries(this.data.nodes).forEach(([k,p])=>{const d=Math.hypot(x-p[0],y-p[1]);if(d<=dist){id=k;dist=d;}});
    if(id===null){
      let n=1;while(this.data.nodes['draw_'+n])n++;id='draw_'+n;
      this.data.nodes[id]=edge ? edge.p : [x,y];
      if(edge) this.data[edge.key].splice(edge.i,1,[edge.a,id],[id,edge.b]);
    }
    if(this.selected && id!==this.selected){
      const a=this.selected,b=id;
      const exists=[...this.data.edges,...this.data.directed_edges].some(e=>(e[0]===a&&e[1]===b)||(e[0]===b&&e[1]===a));
      if(!exists)this.data.edges.push([a,b]);
    }
    this.selected=id;
  }
}
if(typeof module!=='undefined')module.exports=RoadEditor;
