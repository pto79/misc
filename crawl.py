import os
import requests
from lxml import html

headers = {
	#'Host': 'www.zhihu.com',
	#'Accept-Language': 'zh-CN,zh;q=0.8,en;q=0.6',
	'Accept-Encoding': 'gzip, deflate',
	'Connection': 'keep-alive',
	#'Pragma': 'no-cache',
	#'Upgrade-Insecure-Requests': '1',
	'Accept': '*/*',
	'User-Agent': 'Mozilla/5.0 (Machitosh; Intel Mac OS X 10_12_4)'
				  'AppleWebKit/537.36 (KHTML, like Gecko) Chrome.57.0.2987.133 Safari/537.36',
}

def save(text, filename='temp', path='d://app\\image/'):
	fpath = os.path.join('d:\\', 'app', 'download', filename)
	#fpath = 'd://app/image/'
	print(fpath)
	#print(text)
	with open(fpath, 'w') as f:
		f.write(text)


def save_image(image_url):
	resp = requests.get(image_url)
	page = resp.content
	filename = image_url.split('zhimg.com')[-1]
	#print(filename)
	save(page, filename)

def process_url(image_url):
	resp = requests.get(image_url)
	page = resp.content
	root = html.fromstring(page)
	founds = root.xpath('//li')
	#print('founds:', founds)
	for found in founds:
		temp = found.text
		if temp is not None :
			temp = temp.lower();
			if (temp.find("singaporean") != -1 or temp.find("citizen") != -1 ) :
				print('found:', found.text)
				print('link:', image_url)


def crawl(zhihu_url):	
	resp = requests.get(url, headers=headers)
	#print(resp.request.headers)
	page = resp.content
	root = html.fromstring(page)
	image_urls = root.xpath('//tr/td/a[@class="job-link"]/@href')
	#print('count:', len(image_urls))
	#print(image_urls)
	for image_url in image_urls:
		image_url = 'http://careers.pageuppeople.com' + image_url
		process_url(image_url)
		#save_image(image_url)		
	#print(image_urls)
		

if __name__ == '__main__':
	url = 'http://careers.pageuppeople.com/688/cwlive/en/filter/?category=&job-sector=&brand=&search-keyword=software&job-mail-subscribe-privacy=agree&page=1&page-items=200'
	crawl(url)