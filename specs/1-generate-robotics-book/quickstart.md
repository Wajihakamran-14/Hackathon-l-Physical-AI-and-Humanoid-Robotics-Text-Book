# Quickstart: Book Generation

This guide provides the basic steps to initiate and monitor the automated book generation process.

## Prerequisites

1.  **Project Cloned**: The repository is cloned to a local machine.
2.  **Dependencies Installed**: All required Python dependencies for the agent scripts are installed.
3.  **Environment Configured**: Any necessary API keys or environment variables are set.

## Running the Generation Process

The book generation is orchestrated by a main control script, conceptually `run_generation.py`.

### Step 1: Initiate Generation

To start the process from the beginning, run the main script without any arguments:

```bash
python src/agents/run_generation.py
```

The script will:
1.  Read the `plan.md` for the `generate-robotics-book` feature.
2.  Start with Module 1, Chapter 1.
3.  Invoke the Developer Agent to generate the content.
4.  Invoke the System Agent to validate the content.
5.  Save the file to the `/docs` directory.
6.  Repeat for all chapters.

### Step 2: Monitor Progress

The script will provide real-time logging to the console, indicating which chapter is currently being generated or validated.

```text
[INFO] Orchestrator: Starting generation for feature 'generate-robotics-book'...
[INFO] System Agent: Requesting generation for Chapter 1: "Introduction to ROS 2 & Robot Control".
[INFO] Developer Agent: Generating content...
[INFO] Developer Agent: Content for Chapter 1 generated.
[INFO] System Agent: Validating content for Chapter 1...
[INFO] System Agent: Validation passed. Saving to docs/introduction-to-ros-2.md.
[INFO] System Agent: Requesting generation for Chapter 2: "ROS 2 Nodes, Topics, Services Explained".
...
```

### Step 3: Review Final Output

Once the script completes, the full Docusaurus site can be previewed locally.

1.  Navigate to the project root.
2.  Install Docusaurus dependencies:
    ```bash
    npm install
    ```
3.  Start the local development server:
    ```bash
    npm start
    ```
4.  Open a web browser to `http://localhost:3000` to view the generated book.
