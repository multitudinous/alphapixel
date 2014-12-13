/**
 * Scrape Earth Explorer website and get link for a special download option
 * for each scene_id
 *
 * During the Research and Development, I learned that there is no need for
 * a lengthy process and going through login system, just to obtain the link.
 *
 * Link can be obtained directly in one step and that too without login.
 * Isn't that sweet ;)
 *
 * The link is found on webpage that can be located by the following URL schema
 * http://earthexplorer.usgs.gov/download/options/4923/<scene_id>
 * eg. if scene_id = LC80260312014079LGN00
 *     url will be: http://earthexplorer.usgs.gov/download/options/4923/LC80260312014079LGN00
 *
 * CURL will be used to download webpage from the above URL schema.
 * The webpage has button elements for different download options.
 * Each button element has a onclick attribute, which hosts the URL for that download
 * in the form of window.location="<url for the download>"
 *
 * The download option we are interested in is the last one in the group.
 * To get its link we have to simply scrape its onclick attribute.
 *
 * @author  zenista
 * @email   zenista@rocketmail.com
 * @version 1
 * @date    12 Nov, 2014 00:00:00
 * @ide     qt creator
 * @os      ubuntu 14.04 64 bit
 */

// For Win32 static CURL linking
#define CURL_STATICLIB
#pragma comment(lib, "Ws2_32.lib")

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <curl/curl.h>

/**
 * @breif callback to write curl response data to a string
 * @return size_t
 */
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

/**
 * @brief   Downloads HTML Webpage from Earth Explorer Website
 *          This webpage has links for different downloads options.
 *          Among these links, we are only interested in last one.
 * @return  std::String  html web page if download successful, else NOT_FOUND
 */
std::string get_html(std::string scene_id)
{
    //init vars
    CURL *curl;
    CURLcode res;
    std::string readBuffer;
    std::string url = "http://earthexplorer.usgs.gov/download/options/4923/" + scene_id;

    // fire up curl
    curl = curl_easy_init();
    if(curl) {
      // download html with curl
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
      res = curl_easy_perform(curl);
      curl_easy_cleanup(curl);

      return readBuffer;
    } else {
        // no html downloaded
        return "NOT_FOUND";
    }
}


/**
 * @brief   Find all download links in HTML webpage
 * @param   std::string                html
 * @return  std::vector<std::string>   vector of download links
 */
std::vector<std::string> get_all_links(std::string html) {

    // vector collection... that will be returned
    std::vector<std::string> links;

    std::string start_str = "http://earthexplorer.usgs.gov/download/";
    std::string end_str = "'";
    size_t start_index = 0;
    size_t end_index = 0;

    while(1) {
        // find a start and end of a link
        start_index = html.find(start_str, start_index);
        end_index = html.find(end_str, start_index);

        if(start_index != std::string::npos && end_index != std::string::npos) {
            // get link between start and end
            std::string a_link = html.substr(start_index, end_index - start_index);

            // add to vector
            links.push_back(a_link);

            // set start_index > end_index for next loop
            start_index = end_index;
        }
        else
        {
            // either last link, or no link found... break loop
            break;
        }
    }

    return links;
}

/**
 * @brief   Find one link in which we are interested. It is generally the last link in the vector.
 *          But in case finding the interested link, requires more complex algorithm, this function
 *          can be modified appropriately.
 * @param   std::vector<std::string>    vector of links
 * @return  std::string                 the link in which we are interested
 */
std::string get_interested_link(std::vector<std::string> links) {
    std::vector<std::string>::iterator i = links.end();
    std::string link = *--i;
    return link;
}

/**
 * @brief   For a LandsatSceneID get download URL from Earth Explorer
 * @param   std::string    LandsatSceneID
 * @returns std::string    URL
 */
std::string LandsatFetchEarthExplorerURL(std::string LandsatSceneID)
{
    // download html web page for this Landsat scene id
    std::string html = get_html(LandsatSceneID);

    // find all links in the html webpage
    std::vector<std::string> links = get_all_links(html);

    // get the url we are looking for
    if(links.size() > 0) {
        // got it
        return get_interested_link(links);
    } else {
        // nope, not found
        return "";
    }
}

/**
 * testing get_links
 */
void test_get_link() {
    // read test.html to a string buffer
    std::ifstream t("../earthexplorer/test.html");
    std::stringstream buffer;
    buffer << t.rdbuf();
    std::string html = buffer.str();

    // try to find all links from html
    std::vector<std::string> links = get_all_links(html);

    // how many links we got ?
    std::cout << "\n\n\n Links found: " << links.size() << std::endl;

    // print all links
    for(std::vector<std::string>::size_type i=0; i < links.size(); i++)
    {
        std::cout << "\n Link no. " << i << " : " << links[i] << std::endl;
    }

    // print the special link... we are looking for
    std::string result = get_interested_link(links);
    std::cout << "\n\n The link we are interested in is: " <<  result << std::endl;
}


/**
 * @brief main
 * @return  int exit status
 */
int main(void)
{
    std::string scene_id = "LC80260312014079LGN00";
    std::string result = LandsatFetchEarthExplorerURL(scene_id);
    std::cout << "For LandsatSceneID: " << scene_id << " Result is: " << result << std::endl;

    // testing
    // test_get_link();

    return 0;
}
