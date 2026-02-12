# Implementation Plan: Embed Chatbot UI in Existing Docusaurus Book

**Branch**: `004-fastapi-agent-integration` | **Date**: 2025-12-30 | **Spec**: [specs/004-fastapi-agent-integration/spec.md]
**Input**: Feature specification from `/specs/004-fastapi-agent-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a chatbot UI embedded within the existing Docusaurus book that connects to a FastAPI backend. The system extends the Docusaurus UI with a chat interface that communicates with a FastAPI endpoint, which in turn invokes the existing RAG agent logic from agent.py to generate responses with retrieval context.

## Technical Context

**Language/Version**: Python 3.11, Node.js 18+
**Primary Dependencies**: FastAPI, Docusaurus, React, agent.py module, retrieval.py module
**Storage**: N/A (no additional storage needed beyond existing Qdrant)
**Testing**: pytest for backend, Jest for frontend
**Target Platform**: Web browser (Docusaurus frontend), Local development server (FastAPI backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <10 seconds response time for 95% of queries on local development environment
**Constraints**: Must reuse existing agent.py and retrieval.py modules without duplication; Single API endpoint for chat queries; Backend-only implementation with existing Docusaurus frontend
**Scale/Scope**: Single user local development environment

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- Library-first: Reuse existing agent.py and retrieval.py modules without duplication
- CLI Interface: Maintain CLI functionality alongside API
- Test-First: Implement tests for both backend and frontend components
- Integration Testing: Test the full flow from UI to agent and back
- Observability: Log chat interactions for debugging and analytics

## Project Structure

### Documentation (this feature)

```text
specs/004-fastapi-agent-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── api.py               # FastAPI backend with chat endpoint
├── agent.py             # Existing RAG agent logic (to be reused)
└── retrieval.py         # Existing retrieval logic (to be reused)

book/
├── src/
│   ├── components/
│   │   └── Chatbot/     # React components for chatbot UI
│   ├── pages/           # Docusaurus pages (if needed)
│   └── theme/           # Docusaurus theme overrides (if needed)
├── docusaurus.config.ts # Docusaurus configuration (to be updated)
└── package.json         # Frontend dependencies
```

**Structure Decision**: Web application structure with separate backend (FastAPI) and frontend (Docusaurus React components) to enable the chatbot functionality while maintaining separation of concerns.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [No violations identified] | [N/A] |
