from fastapi import APIRouter, HTTPException, Depends, Query
from typing import Optional
from ..services.web_scraper import web_scraper
from ..auth import auth_service
from ..models.user import UserInDB
from ..utils import logger
import asyncio

router = APIRouter()


@router.post("/scrape/", tags=["scraping"])
async def scrape_website(
    url: str,
    max_pages: int = Query(default=10, ge=1, le=100),
    current_user: UserInDB = Depends(auth_service.get_current_user)
):
    """
    Scrape content from a website and add it to the RAG system
    """
    try:
        if not url.startswith(('http://', 'https://')):
            raise HTTPException(status_code=400, detail="URL must start with http:// or https://")

        # Scrape the website
        content_list = await web_scraper.scrape_url(url, max_pages=max_pages)

        if not content_list:
            raise HTTPException(status_code=404, detail="No content found at the specified URL")

        # Process and store the content in the RAG system
        document_id = await web_scraper.process_and_store_content(content_list, url)

        return {
            "document_id": document_id,
            "url": url,
            "pages_scraped": len(content_list),
            "chunks_created": len(content_list),  # Each page becomes chunks
            "status": "completed",
            "message": f"Successfully scraped and indexed content from {url}"
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error scraping website {url}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error scraping website: {str(e)}")


@router.post("/scrape/single-page", tags=["scraping"])
async def scrape_single_page(
    url: str,
    current_user: UserInDB = Depends(auth_service.get_current_user)
):
    """
    Scrape content from a single webpage and add it to the RAG system
    """
    try:
        if not url.startswith(('http://', 'https://')):
            raise HTTPException(status_code=400, detail="URL must start with http:// or https://")

        # Scrape just the single page
        content_list = await web_scraper._fetch_page_content(url)

        if not content_list:
            raise HTTPException(status_code=404, detail="No content found at the specified URL")

        # Process and store the content in the RAG system
        document_id = await web_scraper.process_and_store_content([content_list], url)

        return {
            "document_id": document_id,
            "url": url,
            "title": content_list.get('title', 'Unknown'),
            "status": "completed",
            "message": f"Successfully scraped and indexed content from {url}"
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error scraping single page {url}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error scraping page: {str(e)}")