from setuptools import find_packages, setup
package_name = 'edge_llm_nav'
setup(name=package_name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'llm_task_server=edge_llm_nav.llm_task_server:main',
          'task_graph_verifier=edge_llm_nav.task_graph_verifier:main',
          'task_executor=edge_llm_nav.task_executor:main',
          'execution_monitor=edge_llm_nav.execution_monitor:main',
          'llm_recovery_policy=edge_llm_nav.llm_recovery_policy:main',
          'experiment_logger=edge_llm_nav.experiment_logger:main']})
