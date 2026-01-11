import asyncio
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import time
from typing import List, Dict, Any, Optional
from ..utils import logger
from ..services.document_processor import document_processor
from ..services.qdrant_service import qdrant_service
from ..database import db
import hashlib


class WebScraper:
    def __init__(self):
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        })

    async def scrape_url(self, url: str, max_pages: int = 10) -> List[Dict[str, Any]]:
        """
        Scrape a single URL or a website with multiple pages
        """
        try:
            # Check if it's a single page or a website to crawl
            content = await self._fetch_page_content(url)
            if content:
                return [content]
        except Exception as e:
            logger.error(f"Error scraping single URL {url}: {str(e)}")

        # If it's a website, try to crawl multiple pages
        return await self._crawl_website(url, max_pages)

    async def _fetch_page_content(self, url: str) -> Optional[Dict[str, Any]]:
        """
        Fetch content from a single URL
        """
        try:
            response = self.session.get(url, timeout=30)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Extract title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else urlparse(url).path.split('/')[-1]

            # Extract main content - try different selectors for main content
            main_content = None
            for selector in ['main', 'article', '.content', '#content', '.post', '.entry-content', 'body']:
                main_content = soup.select_one(selector)
                if main_content:
                    break

            if not main_content:
                main_content = soup.body

            if main_content:
                # Get text content
                text_content = main_content.get_text(separator=' ', strip=True)

                # Clean up text
                text_content = ' '.join(text_content.split())

                return {
                    'url': url,
                    'title': title,
                    'content': text_content,
                    'original_html': str(main_content)
                }
        except Exception as e:
            logger.error(f"Error fetching content from {url}: {str(e)}")
            return None

    async def _crawl_website(self, base_url: str, max_pages: int) -> List[Dict[str, Any]]:
        """
        Crawl a website and extract content from multiple pages
        """
        visited_urls = set()
        to_visit = [base_url]
        all_content = []

        while to_visit and len(visited_urls) < max_pages:
            current_url = to_visit.pop(0)

            if current_url in visited_urls:
                continue

            visited_urls.add(current_url)

            # Only process URLs from the same domain
            if self._same_domain(base_url, current_url):
                content = await self._fetch_page_content(current_url)
                if content:
                    all_content.append(content)

                    # Find links on the page to crawl
                    links = await self._extract_links(current_url)
                    for link in links:
                        if link not in visited_urls and self._same_domain(base_url, link):
                            if link not in to_visit:
                                to_visit.append(link)

            # Be respectful with crawling - add delay
            time.sleep(1)

        return all_content

    def _same_domain(self, base_url: str, url: str) -> bool:
        """
        Check if two URLs are from the same domain
        """
        base_domain = urlparse(base_url).netloc
        url_domain = urlparse(url).netloc
        return base_domain == url_domain

    async def _extract_links(self, url: str) -> List[str]:
        """
        Extract all links from a page
        """
        try:
            response = self.session.get(url, timeout=15)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')
            links = []

            for link in soup.find_all('a', href=True):
                href = link['href']
                full_url = urljoin(url, href)

                # Only add URLs that are valid and from the same domain
                if self._is_valid_url(full_url):
                    links.append(full_url)

            return links
        except Exception as e:
            logger.error(f"Error extracting links from {url}: {str(e)}")
            return []

    def _is_valid_url(self, url: str) -> bool:
        """
        Check if URL is valid and not a file download
        """
        parsed = urlparse(url)
        if not parsed.netloc:
            return False

        # Skip common file extensions
        file_extensions = ['.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip', '.exe', '.doc', '.docx']
        return not any(url.lower().endswith(ext) for ext in file_extensions)

    async def process_and_store_content(self, content_list: List[Dict[str, Any]], source_url: str) -> str:
        """
        Process scraped content and store in Qdrant
        """
        try:
            # Combine all content into a single document for processing
            combined_content = []
            for item in content_list:
                combined_content.append(f"Title: {item['title']}")
                combined_content.append(f"URL: {item['url']}")
                combined_content.append(f"Content: {item['content']}")
                combined_content.append("---")  # Separator between pages

            full_content = "\n".join(combined_content)

            # Generate a document ID based on the source URL
            doc_id = hashlib.md5(source_url.encode()).hexdigest()

            # Process the document (chunk and embed)
            processed_chunks = await document_processor.process_document(
                filename=f"web_content_{doc_id[:8]}.txt",
                content=full_content
            )

            # Store embeddings in Qdrant
            point_ids = await qdrant_service.store_embeddings(doc_id, processed_chunks)

            # Store document metadata in database
            document_data = {
                'filename': f"web_content_{doc_id[:8]}.txt",
                'original_name': f"Content from {source_url}",
                'content_type': 'web_content',
                'size_bytes': len(full_content.encode('utf-8')),
                'checksum': hashlib.sha256(full_content.encode()).hexdigest(),
                'status': 'completed',
                'metadata': {
                    'source_url': source_url,
                    'pages_scraped': len(content_list),
                    'scraped_at': time.time()
                }
            }

            document_id = await db.create_document(document_data)

            logger.info(f"Successfully scraped and stored content from {source_url} with {len(processed_chunks)} chunks")

            return document_id

        except Exception as e:
            logger.error(f"Error processing scraped content: {str(e)}")
            raise


# Global web scraper instance
web_scraper = WebScraper()