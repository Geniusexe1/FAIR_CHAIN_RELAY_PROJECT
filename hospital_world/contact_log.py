#!/usr/bin/env python3
# gz topic -e /gazebo/default/physics/contacts | contact_log.py out.jsonl : chain contacts with anything but the floor, one line per 0.5 s + pair
import sys,re,json,time
out=open(sys.argv[1],'a');seen={};c1=c2=None;pos=None;want=False;last=time.time()
def short(n):
    p=n.split('::');return p[0] if p[0]!='chain_robot' else re.sub(r'(_wheel.*)?$','',p[1],count=1) if 'wheel' in p[1] else p[1]
for l in sys.stdin:
  try:
      l=l.strip()
      if l.startswith('collision1:'): c1=l.split('"')[1];pos=None
      elif l.startswith('collision2:'):
          c2=l.split('"')[1];want='ground_plane' not in c1+c2 and 'chain_robot' in c1+c2
          if want:
              k=tuple(sorted((short(c1),short(c2))))
      elif want and pos is None and l.startswith('x:'): pos=[float(l[2:]),None]
      elif want and pos and pos[1] is None and l.startswith('y:'):
          pos[1]=float(l[2:]);seen.setdefault(k,[round(pos[0],2),round(pos[1],2),0]);seen[k][2]+=1
      now=time.time()
      if now-last>.5:
          for k,v in seen.items(): out.write(json.dumps(dict(t=round(now,1),a=k[0],b=k[1],x=v[0],y=v[1],n=v[2]))+'\n')
          out.flush();seen={};last=now
  except Exception as e:
    print("skip", repr(e), file=sys.stderr)
