from glob import glob
from setuptools import find_packages, setup

package_name = "rebotarm_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "fastmcp"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="MCP tool server for safe LLM and voice control of reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_mcp_server = rebotarm_agent.rebotarm_mcp_server:main",
            "rebotarm_text_agent = rebotarm_agent.rebotarm_text_agent:main",
        ],
    },
)
