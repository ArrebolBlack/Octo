import tensorflow as tf
import tensorflow_datasets as tfds

# 指定数据集目录，这里假设数据集存储在 '/home/xiaosa/Newpython/Octo/data_test'
data_dir = '/home/xiaosa/Newpython/Octo/data_test/pybullet_ur5_pick_reset_cup_mug/1.0.0'
# data_dir = '/home/xiaosa/Newpython/Octo/datasets/example_sim_data/aloha_sim_dataset/aloha_sim_cube_scripted_dataset/1.0.0'


# 使用 tfds.builder_from_directory 来构建数据集对象
builder = tfds.builder_from_directory(data_dir)

# 打印数据集的信息
print(builder.info)

# 下载并准备数据集（如果尚未完成）
builder.download_and_prepare()

# 加载数据集
ds = builder.as_dataset(split='train')

# 遍历数据集并打印一些信息
for example in ds.take(1):  # 只查看第一个样本
    for step_data in example['steps']:
        observation = step_data['observation']
        # print('Observation - image_primary:', observation['image_primary'].numpy())
        # print('Observation - proprio:', observation['proprio'].numpy())
        print('Action:', step_data['action'].numpy())
        print('Reward:', step_data['reward'].numpy())
        print('Discount:', step_data['discount'].numpy())
        print('Is first:', step_data['is_first'].numpy())
        print('Is last:', step_data['is_last'].numpy())
        print("language_instruction:", step_data['language_instruction'])
        # print("language_instruction:", step_data['language_instruction'].numpy().decode('utf-8'))
