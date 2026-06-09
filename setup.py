from setuptools import find_packages, setup

package_name = "algorithm_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kestrel",
    maintainer_email="kestrel@inha.edu",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_planning_test = algorithm_test.path_planning_unit_test.path_planning_test:main",
            "path_following_test = algorithm_test.path_following_unit_test.path_following_test:main",
            "path_following_rewp_test = algorithm_test.path_following_unit_test.path_following_rewp_test:main",
            "path_following_bridge_test = algorithm_test.path_following_unit_test.path_following_bridge_test:main",
            "collision_avoidance_test = algorithm_test.collosion_avoidance_unit_test.collision_avoidance_test:main",
            "ca_pf_integrated_test = algorithm_test.ca_pf_integrated_test.ca_pf_integrated_test:main",
            "pp_pf_integrated_test = algorithm_test.pp_pf_integrated_test.pp_pf_integrated_test:main",
        ],
    },
)
