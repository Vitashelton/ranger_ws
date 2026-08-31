from glob import glob
from setuptools import find_packages, setup
package_name = 'edge_llm_nav'
setup(name=package_name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/test_cases', glob('test_cases/*.yaml')),
                  ('share/' + package_name + '/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'llm_task_server=edge_llm_nav.llm_task_server:main',
          'task_graph_verifier=edge_llm_nav.task_graph_verifier:main',
          'task_executor=edge_llm_nav.task_executor:main',
          'execution_monitor=edge_llm_nav.execution_monitor:main',
          'llm_recovery_policy=edge_llm_nav.llm_recovery_policy:main',
          'experiment_logger=edge_llm_nav.experiment_logger:main',
          'run_llm_acceptance=scripts.run_llm_acceptance:main',
          'semantic_grounder=edge_llm_nav.semantic_grounder:main',
          'tool_agent_server=edge_llm_nav.tool_agent_server:main',
          'risk_aware_grounder=edge_llm_nav.risk_aware_grounder:main',
          'rage_nav_gate=edge_llm_nav.rage_nav_gate:main',
          'run_rage_ablation=scripts.run_rage_ablation:main']})
          
