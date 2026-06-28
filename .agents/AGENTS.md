# Workspace Rules

Whenever any file modifications are completed, the agent must commit the changes locally with a descriptive commit message and push them to the configured remote GitHub repository (`main` branch).

Every 5 turns of conversation with the user (e.g., Turn 5, 10, 15, etc.), the agent must append a section at the end of the response titled "### 💡 如何讓問題問得更好？" analyzing the user's recent questions and providing constructive suggestions on how to formulate them more clearly or what additional context to provide.

