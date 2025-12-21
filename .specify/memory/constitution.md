# AI / Spec-Driven Unified Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Development
Every feature and change must originate from explicit specifications (Spec-Kit Plus). No implementation without a validated spec. Specifications serve as the source of truth, not code.

### II. Production-Ready Architecture
All implementations must be designed for production from day one. This includes proper error handling, observability, security considerations, and performance optimization. Features must be deployable and maintainable.

### III. Test-First (NON-NEGOTIABLE)
TDD is mandatory: Tests written → User approved → Tests fail → Then implement; Red-Green-Refactor cycle strictly enforced. All code must have adequate test coverage before merging.

### IV. Integration Testing
Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas. Unit tests alone are insufficient for complex systems.

### V. Observability and Debuggability
All systems must include structured logging, metrics collection, and distributed tracing. Text I/O protocols ensure debuggability. All external dependencies must be monitored.

### VI. Security-First Approach
Security considerations must be integrated from the initial design phase. This includes secure authentication, authorization, data handling, secrets management, and privacy protection.

## Technical Standards

### Code Quality
- DRY: No duplicated logic or documentation
- SOLID: Enforced in all backend and agent code
- Performance-aware: Optimize for latency and cost
- Deterministic outputs for CLI commands where possible

### Documentation Philosophy
- Written using Docusaurus
- Every chapter must map to a spec artifact
- Reproducible setup instructions (no hidden steps)
- All API endpoints must be documented with examples

### RAG Chatbot Architecture Constraints
- Core Stack: OpenAI Agents / ChatKit SDKs, FastAPI backend, Neon Serverless Postgres, Qdrant Cloud
- Answers must be grounded in retrieved documents ONLY
- If no relevant context is found, respond with "Insufficient context"
- No training on user data
- Vector embeddings must be reproducible

## Development Workflow

### CLI Interaction Model
- Input is assumed to be intentional and technical
- Responses must be:
  - Markdown formatted
  - Copy-paste safe
  - CLI-friendly
- Commands must be idempotent where possible
- Explicitly state side effects (DB writes, file creation, deployments)

### Change Management
- All changes must follow the execution contract:
  1. Confirm surface and success criteria (one sentence)
  2. List constraints, invariants, non-goals
  3. Produce the artifact with acceptance checks inlined
  4. Add follow-ups and risks (max 3 bullets)
  5. Create PHR in appropriate subdirectory
  6. Suggest ADRs for architecturally significant decisions

### Quality Assurance
- All technical claims must be verifiable
- Never guess. If data is missing, explicitly request it
- Never hallucinate APIs, SDKs, or library behavior
- No destructive actions without confirmation

## Data Integrity and Privacy

### Security Protocols
- Never expose API keys or secrets
- Use environment variables only
- Validate all external inputs
- Assume all user input may be sensitive
- No logging of raw queries unless required for debugging
- Clear separation between content data and user metadata

### Ethical Standards
- No deceptive behavior
- No fabricated citations
- No misleading technical claims
- Respect user privacy and data ownership

## Governance

All team members must comply with this constitution. Changes to the constitution require formal approval process and must be documented with proper version control. This document supersedes all other practices and serves as the authoritative guide for all development activities.

**Version**: 1.0.0 | **Ratified**: 2025-12-19 | **Last Amended**: 2025-12-19