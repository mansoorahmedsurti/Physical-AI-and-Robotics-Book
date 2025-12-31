---
id: 001
title: Failed to run sp.tasks - PowerShell not found
stage: tasks
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.tasks
labels: ["error", "setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- No files were created or modified.
tests:
- No tests were run or added.
---

## Prompt

/sp.tasks

## Response snapshot

I encountered an error while trying to run the prerequisite check for generating tasks. It appears that PowerShell (`pwsh`) is not found in the environment, which is required to execute the script. I cannot proceed with generating the `tasks.md` file until this is resolved.

## Outcome

- ✅ Impact: Failed to execute tasks command due to missing PowerShell dependency.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: User needs to install PowerShell or provide an alternative to run prerequisite checks.
- 🧠 Reflection: The agent attempted to execute the `/sp.tasks` command, which relies on a PowerShell script for prerequisite checks. The execution failed because `pwsh` was not found in the environment. This blocked further progress on task generation. The agent should inform the user and suggest a resolution.

## Evaluation notes (flywheel)

- Failure modes observed: Missing external dependency (PowerShell).
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure PowerShell is available or provide an alternative prerequisite check mechanism.
