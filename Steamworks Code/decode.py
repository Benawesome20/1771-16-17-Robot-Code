import re
import plotly
from plotly.graph_objs import Scatter, Layout


with open('tele.log', 'r') as myfile:
	data = myfile.read().replace('\n','')
vals = [int(s) for s in re.findall(r'\d+', data)]

times = vals;
times[0::5];
print(times)
# trace = plotly.graph_objs.Scatter(x = )
