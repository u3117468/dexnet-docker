from gqcnn import GQCNN, SGDOptimizer, GQCNNAnalyzer
from autolab_core import YamlConfig

train_config = YamlConfig('training-high-epoch.yaml')
gqcnn_config = train_config['gqcnn_config']

gqcnn = GQCNN(gqcnn_config)

SGDOptimizer = SGDOptimizer(gqcnn, train_config)

with gqcnn.get_tf_graph().as_default():
     SGDOptimizer.optimize()
