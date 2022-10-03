import json
import numpy as np
import open3d as o3d
from time import sleep

def read_json(input_path):
    with open(input_path,'r')  as file:
        str = file.read()
        data = json.loads(str)
    return data
# results = data['results']

def show_graph(data):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    graph_nodes=np.array(data['graph_nodes'])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(graph_nodes,dtype=np.float32))
    # pcd.colors = o3d.utility.Vector3dVector(np.zeros_like(points))   
    # o3d.visualization.draw_geometries([pcd])
    vis.add_geometry(pcd)

    # mesh_origin_graph=get_mesh_origin(graph_data_2[0],graph_data_2[1])

    # vis.add_geometry(mesh_origin_graph[0])
    # vis.add_geometry(mesh_origin_graph[1])
    
    # vis.poll_events()
    # vis.update_renderer()


    ctr = vis.get_view_control()
    # ctr.rotate(90,0.0,0.0)
    ctr.set_lookat(np.array([0, 0,1]))
    ctr.set_up((0, -1, 0))
    ctr.set_front((0, 0, -1))
    ctr.change_field_of_view()


    for i in range(350,400):
        frame=i
        translation_pred_i=np.array(data['data'][str(frame)]['translation_pred'])
        pcd.points = o3d.utility.Vector3dVector(np.asarray(graph_nodes+translation_pred_i,dtype=np.float32))
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        sleep(0.1)

    vis.run()

data = read_json('transform_0350_400.json.json')

#              live data | frame_id | R&T of C model to Live model

rotation_pred=np.array(data['data']['351']['rotation_pred']).reshape(-1,3,3)
print(rotation_pred.shape)

translation_pred=np.array(data['data']['351']['translation_pred'])
print(translation_pred.shape)

graph_nodes=np.array(data['graph_nodes'])
print(graph_nodes.shape)

graph_edges=np.array(data['graph_edges'])
print(graph_edges.shape)

show_graph(data)