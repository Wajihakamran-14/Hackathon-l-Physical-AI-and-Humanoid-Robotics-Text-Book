# Data Model: Technical Robotics Book

This document defines the key data entities for the book generation feature, as extracted from the feature specification.

## Entity Definitions

### 1. Module

-   **Description**: A logical grouping of chapters that represents a major section of the book.
-   **Attributes**:
    -   `title` (string): The name of the module (e.g., "The Robotic Nervous System (ROS 2)").
    -   `module_number` (integer): The sequential order of the module.
-   **Relationships**:
    -   Has many `Chapters`.

### 2. Chapter

-   **Description**: A single Markdown file representing a specific topic within a Module. This is the core content unit of the book.
-   **Attributes**:
    -   `title` (string): The title of the chapter (e.g., "Introduction to ROS 2 & Robot Control").
    -   `chapter_number` (integer): The sequential order of the chapter within the entire book.
    -   `filename` (string): The `lowercase-with-hyphens` filename for the markdown file.
    -   `content` (string): The full Markdown content of the chapter.
-   **Relationships**:
    -   Belongs to one `Module`.
    -   Contains many `Code Examples`.
-   **Validation Rules**:
    -   Must contain the following sections: overview, explanation, code, simulation steps, summary.
    -   Must adhere to Markdown heading hierarchy.

### 3. Code Example

-   **Description**: A formatted block of code embedded within a chapter.
-   **Attributes**:
    -   `language` (string): The language of the code block (e.g., `python`, `xml`, `bash`).
    -   `code` (string): The raw code content.
-   **Relationships**:
    -   Embedded within one `Chapter`.
-   **Validation Rules**:
    -   Code must be syntactically valid for the specified language.
    -   Code must be runnable in the context of the chapter's topic and specified simulation environment.
