import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI


def get_llm():
    """Create and return the configured LLM instance."""
    load_dotenv()

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise ValueError(
            "OPENAI_API_KEY not set. "
            "Set it in your environment or in a .env file."
        )

    model = os.getenv("LLM_MODEL", "gpt-5-mini")
    temperature = float(os.getenv("LLM_TEMPERATURE", "0"))

    return ChatOpenAI(
        model_name=model,
        temperature=temperature,
        openai_api_key=api_key,
    )
