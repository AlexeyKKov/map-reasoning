from distutils.core import setup

with open('requirements.txt') as f:
    required = f.read().splitlines()

setup(
    name='multiMAP',
    version='1.0.0',
    packages=['map-planner', 'map-planner.grounding', 'map-planner.pddl', 'map-planner.search', 'map-planner.agent'],
    package_dir={'map-planner': 'src'},
    url='http://cog-isa.github.io/map-planner/',
    license='',
    author='KiselevGA',
    author_email='kiselev@isa.ru',
    long_description=open('README.md').read(),
    install_requires=required
)