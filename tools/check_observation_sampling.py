import json,sys
j=json.load(sys.stdin)
x=j['foot_control_observations']
assert x['total']==768 and x['stored']==64 and x['truncated']
assert x['attempts']==769 and x['unmatched']==1
assert x['rows'][0][0]==0 and x['rows'][-1][0]==767
assert all(x['rows'][i][0] < x['rows'][i+1][0] for i in range(63))
print('sparse observation sampling spans entire run PASS')
