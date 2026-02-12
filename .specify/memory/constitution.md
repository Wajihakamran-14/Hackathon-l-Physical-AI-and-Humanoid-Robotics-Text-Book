<<<<<<< HEAD
<!--
Sync Impact Report:

- Version change: 0.0.0 -> 1.0.0
- Added sections:
  - Principle: Accuracy
  - Principle: Clarity
  - Principle: Practicality
  - Principle: Modularity
  - Principle: Integration
  - Principle: Consistency
  - Standards and Structure
  - Project Scope and Quality
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md (checked, no changes needed)
  - ✅ .specify/templates/spec-template.md (checked, no changes needed)
  - ✅ .specify/templates/tasks-template.md (checked, no changes needed)
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics — A Technical Book Using Docusaurus Constitution

## Core Principles

### I. Accuracy
Accuracy in robotics, AI, and simulation concepts is paramount. All technical content must be verified and reflect current best practices.

### II. Clarity
Clarity through beginner-friendly technical writing. We prioritize simple English, short paragraphs, and a step-by-step approach to make complex topics accessible.

### III. Practicality
Practicality using real ROS 2, Gazebo, Unity, and Isaac Sim workflows. Examples must be functional, illustrative, and grounded in real-world applications.

### IV. Modularity
Each chapter is a standalone learning unit. Readers should be able to dive into any chapter and gain value without having read the preceding ones, though a logical flow will exist.

### V. Integration
Integration of AI + robotics (embodied intelligence). The book's core theme is bridging AI "brains" with physical "bodies," and all content should serve this narrative.

### VI. Consistency
Consistency with all SpecKitPlus specs and structure. Adherence to the defined project structure, file naming, and formatting is mandatory.

## Key Standards & Book Structure

### Writing Style and Formatting
- **Language**: Simple English, short paragraphs.
- **Workflow**: All tutorials must be presented as step-by-step workflows.
- **Code**: Real, runnable code examples are required for all concepts.
- **Visuals**: Diagrams (ASCII/Mermaid) are allowed and encouraged for clarity.
- **Structure**: Each chapter must include an overview, clear explanations, code samples, simulation steps, and a summary.
- **Technology**: All content must be in Markdown and follow the Docusaurus structure. Filenames must be `lowercase-with-hyphens`. Code blocks must be language-labeled (e.g., python, xml, bash).

### Book Structure Requirements
The book is divided into four modules, each with at least three chapters:
- **Module 1: The Robotic Nervous System (ROS 2)**: Covers nodes, topics, services, `rclpy`, URDF, and control pipelines.
- **Module 2: The Digital Twin (Gazebo & Unity)**: Covers physics simulation, sensors, and human-robot interaction.
- **Module 3: The AI-Robot Brain (NVIDIA Isaac)**: Covers Isaac Sim, synthetic data, Isaac ROS, and Nav2 integration.
- **Module 4: Vision-Language-Action (VLA)**: Covers voice-to-action, LLM-based planning, and the capstone autonomous humanoid project.

## Project Scope & Quality

### Constraints & Success Criteria
- **Constraints**: Minimum 12 chapters; examples must run on specified open-source tools (ROS 2 Humble, Gazebo, etc.); original content only.
- **Success Criteria**: All four modules are complete and the code is runnable. The Docusaurus site must build successfully and be deployable to GitHub Pages.

### Non-Goals
- No hardware-specific tutorials for paid humanoids (simulation only).
- No deep mathematical theory.
- No advanced LLM research theory (focus is on applied VLA).
- No complex 3D mesh modeling.

### Quality Benchmarks
- Readability aimed at grade 8–10 level.
- High technical accuracy is non-negotiable.
- Smooth logical flow between modules.
- Professional, consistent formatting.

## Governance
This Constitution is the single source of truth for project principles and standards. All contributions, reviews, and architectural decisions must align with it. Amendments require a documented proposal, review, and an update to the version number and date below.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
=======
# [PROJECT_NAME] Constitution
<!-- Example: Spec Constitution, TaskFlow Constitution, etc. -->

## Core Principles

### [PRINCIPLE_1_NAME]
<!-- Example: I. Library-First -->
[PRINCIPLE_1_DESCRIPTION]
<!-- Example: Every feature starts as a standalone library; Libraries must be self-contained, independently testable, documented; Clear purpose required - no organizational-only libraries -->

### [PRINCIPLE_2_NAME]
<!-- Example: II. CLI Interface -->
[PRINCIPLE_2_DESCRIPTION]
<!-- Example: Every library exposes functionality via CLI; Text in/out protocol: stdin/args → stdout, errors → stderr; Support JSON + human-readable formats -->

### [PRINCIPLE_3_NAME]
<!-- Example: III. Test-First (NON-NEGOTIABLE) -->
[PRINCIPLE_3_DESCRIPTION]
<!-- Example: TDD mandatory: Tests written → User approved → Tests fail → Then implement; Red-Green-Refactor cycle strictly enforced -->

### [PRINCIPLE_4_NAME]
<!-- Example: IV. Integration Testing -->
[PRINCIPLE_4_DESCRIPTION]
<!-- Example: Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas -->

### [PRINCIPLE_5_NAME]
<!-- Example: V. Observability, VI. Versioning & Breaking Changes, VII. Simplicity -->
[PRINCIPLE_5_DESCRIPTION]
<!-- Example: Text I/O ensures debuggability; Structured logging required; Or: MAJOR.MINOR.BUILD format; Or: Start simple, YAGNI principles -->

### [PRINCIPLE_6_NAME]


[PRINCIPLE__DESCRIPTION]

## [SECTION_2_NAME]
<!-- Example: Additional Constraints, Security Requirements, Performance Standards, etc. -->

[SECTION_2_CONTENT]
<!-- Example: Technology stack requirements, compliance standards, deployment policies, etc. -->

## [SECTION_3_NAME]
<!-- Example: Development Workflow, Review Process, Quality Gates, etc. -->

[SECTION_3_CONTENT]
<!-- Example: Code review requirements, testing gates, deployment approval process, etc. -->

## Governance
<!-- Example: Constitution supersedes all other practices; Amendments require documentation, approval, migration plan -->

[GOVERNANCE_RULES]
<!-- Example: All PRs/reviews must verify compliance; Complexity must be justified; Use [GUIDANCE_FILE] for runtime development guidance -->

**Version**: [CONSTITUTION_VERSION] | **Ratified**: [RATIFICATION_DATE] | **Last Amended**: [LAST_AMENDED_DATE]
<!-- Example: Version: 2.1.1 | Ratified: 2025-06-13 | Last Amended: 2025-07-16 -->
>>>>>>> 003-openai-agent-rag
