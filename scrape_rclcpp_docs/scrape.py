from bs4 import BeautifulSoup
import bs4
import os
import urllib.request
import re
import json
import copy
url = "http://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Node.html"
print(os.path.basename(url))



#print(lines)

def countNumChars(mystr, char):
  count = 0
  for c in mystr:
    if c == char:
      count += 1
  return count

def removeSpacesBeforeAngleBrackets(astring):
  return re.sub(r" >", r">", re.sub(r"< ", r"<", astring))

def putBracketsBackTogether(splits, brackL, brackR, splitter=" "):
  chooseIndices = []
  ignoreIndices = []

  def countLefts(mystr):
    return countNumChars(mystr, brackL)
  def countRights(mystr):
    return countNumChars(mystr, brackR)

  def counts(mystr):
    return (countLefts(mystr), countRights(mystr))
  
  for i in range(len(splits)):
    if i not in ignoreIndices:
      split = splits[i]
      langleCount, rangleCount = counts(split)
      print(langleCount, rangleCount)
      chooseIndices.append(i)
      if langleCount != rangleCount:
        
        oldI = i
        while langleCount != rangleCount and i < len(splits) - 1:
          split = split + splitter + splits[i + 1]
          langleCount, rangleCount = counts(split)
          i += 1
          ignoreIndices.append(i)
        print("New split: " + split)
        splits[oldI] = split
      
  return [splits[i] for i in range(len(splits)) if i in chooseIndices]

def putArgsBackTogether(args):
  return [removeSpacesBeforeAngleBrackets(arg) for arg in putBracketsBackTogether(args, "<", ">", ", ")]


def getJsonDict(args, returnType, func_type, template, include, ref):
  jsonDict = {
    "args": args,
    "return": returnType,
    "func_type": func_type,
    "template": template,
    "include": include,
    "ref": ref
  }
  if func_type == "":
    jsonDict.pop("func_type")
  if include == "":
    jsonDict.pop("include")
  if len(template) == 0:
    jsonDict.pop("template")
  return jsonDict

def hasClass(tag, classId):
  return "class" in tag.attrs and classId in tag["class"]

def getIncludeString(soup):
  include = ""
  codes = [code for code in soup("code") if code.getText().find("include") > -1]
  if (len(codes)) > 0:
    print("Length of codes array is " + str(len(codes)))
    if (len(codes)) == 1:
      mystr = codes[0].getText().strip()
      include = re.sub(r"#include\s* <([^>]+)>", r"\1", mystr)
      print("The include: " + include)
  return include

def getArrayType(arg):
  if arg.find("[") < 0:
    return ""
  firstIndex = arg.find("[")
  lastIndex = arg.find("]")
  return " " + arg[firstIndex:(lastIndex + 1)]

def hasClassStartingWith(tag, classPrefix):
  if "class" not in tag.attrs:
    return False
  classes = tag["class"]
  for c in classes:
    if c.startswith(classPrefix):
      return True
  return False

def getSoup(url):
  lines = ""
  with urllib.request.urlopen(url) as f:
    lines = f.read().decode("utf-8")
  signatures = {}
  soup = BeautifulSoup(lines, 'lxml')
  return soup

def getNamespace(soup):
  divs = soup("div")
  namespace = ""
  if len(divs) > 0:
    divs = [div for div in divs if hasClass(div, "title")]
    print("len of divs: " + str(len(divs)))
    if (len(divs) == 1):
      mydiv = divs[0]
      print("text: " + mydiv.getText().strip())
      txt = mydiv.getText().strip()
      namespace = removeSpacesBeforeAngleBrackets(putBracketsBackTogether(txt.split(" "), "<", ">")[0])
      #re.sub(r"([a-zA-Z:_]*) .*", r"\1", mydiv.getText().strip())
      print("namespace: " + namespace)
  return namespace

def isClass(soup):
  divs = soup("div", class_="title")
  print("len of divs: {}".format(len(divs)))

  if len(divs) == 1:
    mydiv = divs[0]
    print("text: " + mydiv.getText().strip())
    txt = mydiv.getText().strip()
    return txt.find("Class") > -1
  return False

def getFunctionsTable(soup):
  tables = soup("table")
  print(len(tables))
  declTable = None
  for tab in tables:
    if "class" in tab.attrs:
      if "memberdecls" in tab["class"]:
        #print("Found table with class memberdecls")
        rows = tab("tr")
        if len(rows) > 0:
          first = rows[0]
          if first.getText().strip().find("Functions") > -1:
            return tab
  return None

def getClassFunctionsFromRows(rows):
  return [row for row in rows[1:] if hasClassStartingWith(row, "memitem") and not hasClass(row, "inherit")]

def getGroupHeaders(soup):
  return [header for header in soup("h2") if hasClass(header, "groupheader")]

def hasTemplateParams(row):
  if isinstance(row, bs4.NavigableString):
    return False
  tparamRow = row.find_previous_sibling("tr")

  if hasClassStartingWith(tparamRow, "separator") or hasClass(tparamRow, "heading"):
    return False
    
  #print("Previous sibling: {}".format(tparamRow))

  #if tparamRow is not None and (not isinstance(tparamRow, bs4.NavigableString)) and hasClassStartingWith(tparamRow, "memitem") and tparamRow.find("td", class_="memTemplParams") is not None:
    #print("Look at this row: {}".format(tparamRow.getText()))
  
  return (tparamRow is not None) and not isinstance(tparamRow, bs4.NavigableString) and hasClassStartingWith(tparamRow, "memitem") and tparamRow.find("td", class_="memTemplParams") is not None


def getTemplate(row):
  templateRow = row.find_previous_sibling("tr")

  if hasClassStartingWith(templateRow, "memitem") and templateRow.find("td", class_="memTemplParams") is not None:
    return extractTemplate(templateRow.getText())
  return None

def getPublicFunctionNames(soup):
  declTable = getFunctionsTable(soup)
  funcNames = []
  functionTemplates = {}
  
  if declTable:
    print("declTable does have memberdecls class")
    rows = declTable("tr")
    first = rows[0]
    if hasClass(first, "heading"):
      if first.getText().strip().startswith("Public") or first.getText().strip().startswith("Functions"):
        rest = getClassFunctionsFromRows(rows)
        print("len of rest: " + str(len(rest)))

        for row in rest:
          cells = row("td")
          if len(cells) == 2:
            #print("Two cells in this row: " + str(cells))
            cellLeft = cells[0]
            cellRight = cells[1]
            if (hasClass(cellLeft, "memItemLeft") and hasClass(cellRight, "memItemRight")) or (hasClass(cellLeft, "memTemplItemLeft") and hasClass(cellRight, "memTemplItemRight")):
              returnType = removeSpacesBeforeAngleBrackets(cellLeft.getText().strip())
              if (returnType != "virtual"):
                func_name = cellRight.contents[0].getText().strip()
                #print("function name: " + func_name)
                funcNames.append(func_name)

                if hasTemplateParams(row):
                  
                  functionTemplates[func_name] = getTemplate(row)
    else:
      print("declTable does not have memberdecls class.")
  print(functionTemplates)
  return funcNames, functionTemplates
  
def isSoupString(soupElmt):
  return isinstance(soupElmt, bs4.NavigableString)

def getWantedHeaders(headers, wanted):
  return [header for header in headers if header.string in wanted]

def getNextSiblingsOfHeader(hdr):
  sibs = []
  print("Finding siblings of header: {}".format(hdr))
  sib = hdr.find_next_sibling("h2")
  while sib is not None and not hasClass(sib, "groupheader"):
    if hasClass(sib, "memtitle"):
      sibs.append(sib)
      #print("Adding sib: {}".format(sib))
    sib = sib.find_next_sibling("h2")
  #return [sib for sib in hdr.find_next_siblings("h2") if hasClass(sib, "memtitle")]
  return sibs

def getImmediateSiblingsOfHeaders(headers):
  return [getNextSiblingsOfHeader(hdr) for hdr in headers]

def getUniqueElmts(elmtList):
  newList = []
  for elmt in elmtList:
    if elmt not in newList:
      newList.append(elmt)
  return newList

def extractTemplate(templateString):
  txt = re.sub(r"typename|class", r"", templateString)
  template = []
  begin = txt.find("<")
  begin = begin + 1 if begin > -1 else len("template<")
  end = txt.rfind(">")
  end = end if end > -1 else len(txt) - 1
  txt = txt[begin:end]
  #print("")
  #print("template: " + txt)
  #print("")
  splits = putBracketsBackTogether(txt.split(","), "<", ">", ",")
  #splits = txt.split(", typename")
  #print(splits)

  splits = [split.strip() for split in splits]
  
  typename = "typename"
  klass = "class"
  equals = "="
  for i in range(len(splits)):
    string = splits[i].strip()
    if string.startswith(typename):
      print("old string: {}".format(string))
      string = string[len(typename):].strip()
      print("new string: {}".format(string))
    if string.startswith(klass):
      print("old string: {}".format(string))
      string = string[len(klass):].strip()
      print("new string: {}".format(string))
    if string.find(equals) > -1:
      stringSplits = string.split(equals)
      if len(stringSplits) == 2:
        template.append({
          "name": stringSplits[0].strip(),
          "value": re.sub(r"^::", r"", stringSplits[1].strip())
        })
    else:
      template.append({
        "name": string
      })
  return template

def extractSignature(div, func_name, func_permalink, url, base_url, include, namespace, templateMaybe=None):
  template = []
  if templateMaybe is None:
    divTemplate = div.find("div", class_="memtemplate")
    if divTemplate:
      #print(divTemplate)

      txt = divTemplate.getText().strip()
      template = extractTemplate(txt)
      #print(template)
      #print("")
  else:
    print("The function with name {} has a template.".format(func_name))
    template = templateMaybe
  sigTable = div.find("table", class_="memname")
  params = []
  if sigTable is not None:
    params = sigTable("td", class_=["paramtype", "paramname"])
  args = []

  externalRefs = getUniqueElmts([a.attrs["href"] for a in div("a", class_="elRef", href=True) if a is not None])
  #print("externalRefs: " + str(externalRefs))
  rosRefs = getUniqueElmts([base_url + a.attrs["href"] for a in div("a", class_="el", href=True) if a is not None])

  returnType = ""
  nameAndRetType = div.find("td", class_="memname")
  if nameAndRetType is not None:
    txt = re.sub(r", ", r",", removeSpacesBeforeAngleBrackets(nameAndRetType.getText().strip()))
    #print("<{}>".format(txt))
    lastSpace = txt.rfind(" ")
    if lastSpace > -1:
      returnType = txt[0:lastSpace]
  func_type = ""
  if returnType == "":
    func_type = "constructor"
  else:
    returnType = removeSpacesBeforeAngleBrackets(returnType)
  #print("ros refs: " + str(rosRefs))
  
  if len(params) % 2 == 0 and len(params) > 0:
    oldLen = len(params)
    params = [(params[2 * i], params[2 * i + 1]) for i in range(len(params) // 2)]
    #print("Params length makes sense: " + str(len(params) * 2 == oldLen))
    #print(params)
    for tipe, name in params:
      #print(tipe, name)
      arg = tipe.getText().strip() + getArrayType(name.getText())
      code = name.find("code")
      if code is not None and name.getText().find("=") > -1:
        codeText = code.getText().strip()
        if codeText.startswith("(") and codeText.endswith(")"):
          codeText = codeText[1:(len(codeText) - 1)]
        arg = arg + " (=" + codeText + ")"
      args.append(removeSpacesBeforeAngleBrackets(arg))
  #print(args)
  if len(args) == 0:
    args = ["void"]

  return getJsonDict(args, returnType, func_type, template, include, getUniqueElmts([url + func_permalink] + externalRefs + rosRefs))

def getPermalink(header2):
  # this should be an instance of a <h2> tag
  if header2.name == "h2":
    if hasClass(header2, "memtitle"):
      if len(header2("a", href=True)) >= 1:
        return header2.find("a", href=True).attrs["href"]
  return "<ERROR: SHOULD NOT BE TRYING TO GET A PERMALINK FROM THIS OBJECT>"

def getNumIters(header2):
  return 1 if len(header2.contents) != 3 else int(re.sub(r"\[[0-9]+/([0-9]+)\]", r"\1", header2.contents[2].string))

def getFunctionName(header2):
  return re.sub(r"\(\)", r"", header2.contents[1].strip()).strip()

def isNotDeconstructor(header2):
  return not header2.contents[1].strip().startswith("~")

#def getSignaturesForFunction(functionHeader, 

def addNamespace(tipe, namespace, originalTypedefs):
  def quantify(name):
    #print("Name: {}, name in original typedefs: {}".format(name, name in originalTypedefs))
    name = re.sub(r"typename|class", r"", name).strip()
    sep = "::" if len(namespace) > 0 else ""
    if name.find("::") > -1:
      firstName = name[:name.find("::")]
      if firstName in originalTypedefs:
        return namespace + sep + name
    if name in originalTypedefs:
      return namespace + sep + name
    elif name.find("Alloc") > -1:
      return name
    elif (not name.endswith("T")) and name.find("allocator::") == -1 and name.find("std::") == -1 and name.find("rclcpp::") == -1 and name.lower() != name:
      return "rclcpp::" + name
    elif name.find("allocator::") > -1:
      return "rclcpp::" + name
    return name
  def cleanParams(string, a, b):
    params = string[a + 1:b].split(",")
    return list(map(lambda x: re.sub(r"typename|class", r"", addNamespace(x.strip(), namespace, originalTypedefs)).strip(), params))
    
  
  firstAngle = tipe.find("<")
  lastAngle = tipe.rfind(">")
  if firstAngle != -1 and lastAngle != -1:
    origType = re.sub(r"typename|class", r"", tipe[:firstAngle]).strip()
    params = tipe[firstAngle + 1:lastAngle].split(",")
    last = tipe[lastAngle + 1:] if lastAngle + 1 < len(tipe) else ""
    params = list(map(lambda x: re.sub(r"typename|class", r"", addNamespace(x.strip(), namespace, originalTypedefs)), params))
    origType = quantify(origType)
    """if origType.find("std::") == -1 and origType.find("rclcpp::") == -1
      origType = namespace + "::" + origType"""
    return origType + "<" + ", ".join(params) + ">" + last
  elif tipe.find("(") != -1 and tipe.rfind(")") != -1:
    firstParen = tipe.find("(")
    lastParen = tipe.rfind(")")
    origType = quantify(re.sub(r"typename|class", r"", tipe[:firstParen]).strip())
    params = cleanParams(tipe, firstParen, lastParen)
    last = tipe[lastParen + 1:] if lastAngle + 1 < len(tipe) else ""
    return origType + "(" + ", ".join(params) + ")" + last
  #tipe = quantify(tipe)
  """if tipe.find("std::") == -1:
    tipe = namespace + "::" + tipe"""
  return quantify(tipe)

def getTypedefs(url):
  typedefs = {}
  soup = getSoup(url)
  include = getIncludeString(soup)
  namespace = getNamespace(soup)
  groups = getGroupHeaders(soup)
  wanted = soup("a", attrs={"name": ["typedef-members", "pub-types"]})
  template = []
  tipesProvided = []

  def startsWithTemplate(htmlElmt):
    return htmlElmt.getText().strip().startswith("template") if not isinstance(htmlElmt, bs4.NavigableString) else htmlElmt.strip().startswith("template")
  
  h3s = soup("h3")
  for h3 in h3s:
    if startsWithTemplate(h3):
      for child in h3.children:
        if startsWithTemplate(child):
          template = extractTemplate(child)
          print("Our template: {}".format(template))
          break
      break
  
  
  if isClass(soup):
    tipesProvided.append(namespace)
    
  print("Wanted:", wanted)
  if (len(wanted) == 1):
    fakeTypedefs = {}
    header = wanted[0]
    table = header.find_parent("table", class_="memberdecls")

    rows = [row.find("td", class_="memItemRight") for row in table("tr") if hasClassStartingWith(row, "memitem")]
    
    for tr in rows:
      text = tr.getText()
      print(text)
      splits = [s.strip() for s in text.split("=")]
      if len(splits) == 2:
        #print("Adding ({}, {}) to the typedefs".format(splits[0], splits[1]))
        fakeTypedefs[splits[0]] = removeSpacesBeforeAngleBrackets(splits[1])
        #typedefs[namespace + "::" + splits[0]] = addNamespace(removeSpacesBeforeAngleBrackets(splits[1]), namespace)
      else:
        print("Too many/few splits: {}", str(splits))

    for tipe,definition in fakeTypedefs.items():
      newKey = namespace + "::" + tipe
      typedefs[newKey] = addNamespace(definition, namespace, fakeTypedefs)
      tipesProvided.append(newKey)
  print("Template: {}".format(template))
  return {include: {"defs": typedefs, "types": tipesProvided}} if len(template) == 0 else {include: {"defs": typedefs, "types": tipesProvided, "template": template}}
  

def getSignatures(url):
  base_url = url[:(-len(os.path.basename(url)))]
  signatures = {}
  soup = getSoup(url) # BeautifulSoup(lines, 'lxml')
  include = getIncludeString(soup)
  namespace = getNamespace(soup)
  declTable = getFunctionsTable(soup)
  headers = getGroupHeaders(soup)
  wantedHeaders = ["Constructor & Destructor Documentation", "Member Function Documentation", "Function Documentation"]
  headers = getWantedHeaders(headers, wantedHeaders)
  print(headers)

  def extract(div, functionName, permanentLink, isConstructor, templateMaybe=None):
    extracted = extractSignature(div, functionName, permanentLink, url, base_url, include, namespace, templateMaybe)
    if isConstructor:
      extracted["func_type"] = "constructor"
    return extracted

  constructorBlockTitles = ["Constructor & Destructor Documentation"]

  functionNames, functionTemplates = getPublicFunctionNames(soup)
  print(functionNames)
  
  headerSibs = getImmediateSiblingsOfHeaders(headers)
  
  for sibs in headerSibs:
    index = headerSibs.index(sibs)
    isConstructor = False
    if headers[index].string in constructorBlockTitles:
      isConstructor = True
    #print("Number of sibs for header {}: {}".format(headers[index], len(sibs)))
    skip = []
    for sib in sibs:
      sibIndex = sibs.index(sib)
      if sibIndex not in skip:
        #print("Contents size: " + str(len(sib.contents)))
        funcName = getFunctionName(sib)
        permalink = getPermalink(sib) #sib.find("a", href=True).attrs["href"]
        
        #print("function name: " + funcName)
        if funcName in functionNames:
          funcKey = namespace + "::" + funcName
          numIters = getNumIters(sib)
          divSibling = sib.find_next_sibling("div", class_="memitem")
          if numIters == 1:
            signatures[funcKey] = extract(divSibling, funcName, permalink, isConstructor)
            if isConstructor:
              signatures[funcKey]["func_type"] = "constructor"
          else:
            it = 0
            #print("Iterating over key <{}>".format(funcKey))
            signatures[funcKey] = []
            while len(signatures[funcKey]) < numIters and divSibling is not None:
              #print("sibling of {}: {}".format(sib, divSibling))
              title = divSibling.find_previous_sibling("h2", class_="memtitle")
              skip.append(sibIndex + it)
              #print("Skipping {}".format(title))
              if isNotDeconstructor(title):
                signatures[funcKey].append(extract(divSibling, funcName, getPermalink(title), isConstructor, functionTemplates[funcName] if funcName in functionTemplates else None))
              it += 1
              
              divSibling = divSibling.find_next_sibling("div", class_="memitem")
  return signatures

def addKeyToTags(tags, category, name):
  if category not in tags:
    tags[category] = []
  if name not in tags[category]:
    tags[category].append(name)

def stringContains(theString, containedSubstring):
  return theString.find(containedSubstring) > -1

def inNamespace(functionName, namespace):
  return (len(functionName) > len(namespace)) and (functionName.find(namespace) > -1)

def getBaseFunctionName(functionName):
  if functionName.find(":") > -1:
    startIndex = functionName.rfind(":") + 1
    return functionName[startIndex:]
  return "SORRY"

def hasExtras(name):
  return name in ["node"]

def hasMakeShared(name):
  return name in ["node", "publisher", "subscription"]

def replaceAllInDictionary(find, replace, dic):
  if isinstance(dic, dict):
    for key in dic:
      newKey = re.sub(find, replace, key)
      dic[key] = replaceAllInDictionary(find, replace, dic.pop(key))
    return dic
  elif isinstance(dic, str):
    return re.sub(find, replace, dic)
  elif isinstance(dic, list):
    for i in range(len(dic)):
      dic[i] = replaceAllInDictionary(find, replace, dic[i])
    return dic
  else:
    print("WARNING: DID NOT GET A DICTIONARY, STRING, or ARRAY: {}".format(dic))

def isConstructorMethod(sig):
  ftKey = "func_type"
  conKey = "constructor"
  if isinstance(sig, list):
    for s in sig:
      if ftKey not in s or s[ftKey] != "constructor":
        return False
    return True
  else:
    return ftKey in sig and sig[ftKey] == "constructor"

def isRelatedTo(funcName, sig, someKey):
  if funcName.lower().find(someKey) > -1:
    return True
  if isinstance(sig, list):
    for s in sig:
      if s["return"].find(someKey) == -1:
        return False
    return True
  return sig["return"].find(someKey) > -1

    
urlDict = {
  "node": "http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html",
  "publisher": "http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Publisher.html",
  "subscription": "http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Subscription.html",
  "rclcpp": "http://docs.ros2.org/latest/api/rclcpp/namespacerclcpp.html",
  "rate": "http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1GenericRate.html"
}

mydir = "jsons"

if (not os.path.isdir(mydir)):
  os.mkdir(mydir)

sigs = {}

hppFilesPrefix = {
  "node": "rclcpp/",
  "messages": "",
  "publisher": "rclcpp/",
  "subscription": "rclcpp/",
  "rate": "rclcpp/",
  "rclcpp": "rclcpp/rclcpp.hpp"
}

classNames = {
  "node": "Node",
  "publisher": "Publisher<MessageT, Alloc>",
  "subscription": "Subscription<CallbackMessageT, Alloc>"
}

classConstructor = {
  "node": "rclcpp::Node::Node",
  "publisher": "rclcpp::Publisher<MessageT, Alloc>::Publisher",
  "subscription": "rclcpp::Subscription<CallbackMessageT, Alloc>::Subscription"
}

typeDefinitions = {}

def addClassTemplate(funcsig, templateName):
  print("Looking at {} for {}".format(templateName, funcsig))
  def addHelper(actualSig):
    if "template" in actualSig:
      print("Template in {}".format(actualSig))
      for i in range(len(actualSig["template"])):
        temp = actualSig["template"][i]
        if temp["name"] == templateName:
          temp["is_class_template"] = "true"
        actualSig["template"][i] = temp
    return actualSig
    
  if isinstance(funcsig, list):
    for i in range(len(funcsig)):
      funcsig[i] = addHelper(funcsig[i])
  else:
    funcsig = addHelper(funcsig)
  return funcsig

for name,url in urlDict.items():
  signatures = getSignatures(url)
  tipeDefs = getTypedefs(url)
  print(tipeDefs)
  for key in tipeDefs:
    if name in hppFilesPrefix:
      typeDefinitions[hppFilesPrefix[name] + key] = tipeDefs[key]
    else:
      typeDefinitions[key] = tipeDefs[key]
    if "template" in tipeDefs[key]:
      print("Template in tipeDefs: {}".format(tipeDefs))
    
      for func, funcsig in signatures.items():
        for t in tipeDefs[key]["template"]:
          signatures[func] = addClassTemplate(funcsig, t["name"])
  if name == "rclcpp":
    for func,funcsig in signatures.items():
      if isinstance(funcsig, list):
        for sig in funcsig:
          sig["include"] = "rclcpp/rclcpp.hpp"
      else:
        funcsig["include"] = "rclcpp/rclcpp.hpp"
  else:
    for func,funcsig in signatures.items():
      if isinstance(funcsig, list):
        for sig in funcsig:
          if "include" in sig:
            sig["include"] = "rclcpp/" + sig["include"]
      else:
        if "include" in funcsig:
          funcsig["include"] = "rclcpp/" + funcsig["include"]

  """if hasMakeShared(name):
    with open("make-shared.json", "r") as f:
      makeShrd = replaceAllInDictionary(r"#CLASS#", classNames[name], json.load(f))
      for key in makeShrd:
        if key not in signatures:
          signatures[key] = makeShrd[key]"""
      
  if hasExtras(name):
    with open("{}_extras.json".format(name), "r") as f:
      extra = json.load(f)
      for key in extra:
        if key not in signatures:
          signatures[key] = extra[key]
  sigs[name] = signatures
  print(signatures)
  with open(os.path.join(mydir, name + ".json"), "w") as f:
    f.write(json.dumps(signatures, indent=4))


typeDefinitions["std_msgs/msg/string.hpp"] = {
  "defs": {},
  "types": [
    "std_msgs::msg::String"
  ]
}

# Write types to a file
with open(os.path.join(mydir, "types.json"), "w") as f:
  f.write(json.dumps(typeDefinitions, indent=4))
    


fileJsons = ["message", "duration"]

for js in fileJsons:
  with open("jsons/{}.json".format(js), "r") as f:
    jsonOnly = json.load(f)
    sigs[js] = jsonOnly
    print(jsonOnly)




tags = {
  "tag_to_sigs": {},
  "sig_to_tags": {}
}
t2s = "tag_to_sigs"
s2t = "sig_to_tags"
conTag = "constructor"
otherConTag = "create_"
initTag = "initialization"
subTag = "subscription"
otherSubTag = "subscri"
pubTag = "publisher"
spinTag = "spin"
shutTag = "shutdown"
sleepTag = "sleep"
okTag = "ok"
publishTag = "publish"
messageTag = "messages"
dataTag = "data"

def getContainsPredicate(string):
  return lambda fName, mSig: stringContains(fName, string)

def getRelatedToPredicate(tag):
  return lambda fName, mSig: isRelatedTo(fName, mSig, tag)

predicateTags = {}

predicateTags[conTag] = [
  getRelatedToPredicate(otherConTag), #lambda fName, mSig: isRelatedTo(fName, mSig, otherConTag),
  lambda fName, mSig: isConstructorMethod(mSig)
]
predicateTags[initTag] = [
  lambda fName, mSig: stringContains(fName, "init") and not stringContains(fName, "is_initialized")
]
predicateTags[subTag] = [
  getRelatedToPredicate(subTag),
  getRelatedToPredicate(otherSubTag)
  #lambda fName, mSig: isRelatedTo(fName, mSig, subTag),
  #lambda fName, mSig: isRelatedTo(fName, mSig, otherSubTag),
]
predicateTags[pubTag] = [
  getRelatedToPredicate(pubTag)
  #lambda fName, mSig: isRelatedTo(fName, mSig, pubTag)
]
predicateTags[spinTag] = [
  getContainsPredicate(spinTag)
  #lambda fName, mSig: stringContains(fName, spinTag)
]
predicateTags[shutTag] = [
  lambda fName, mSig: stringContains(fName, shutTag) and not stringContains(fName, "on_" + shutTag)
]
predicateTags[sleepTag] = [
  lambda fName, mSig: stringContains(fName, sleepTag)
]
predicateTags[okTag] = [
  lambda fName, mSig: stringContains(fName, okTag) and stringContains(fName, "::" + okTag)
]
predicateTags[publishTag] = [
  lambda fName, mSig: inNamespace(fName, "rclcpp::Publisher") and stringContains(getBaseFunctionName(fName), publishTag)
]
predicateTags[messageTag] = [
  lambda fName, mSig: inNamespace(fName, "std_msgs::msg")
]
predicateTags[dataTag] = [
  lambda fName, mSig: inNamespace(fName, "std_msgs::msg") and stringContains(fName, dataTag)
]


for tagName,signatures in sigs.items():
  for funcName, methodSignature in signatures.items():
    addKeyToTags(tags[t2s], tagName, funcName)
    #if tagName not in tags[t2s]:
    #  tags[t2s][tagName] = []
    #if funcName not in tags[t2s][tagName]:
    #  tags[t2s][tagName].append(funcName)
    addKeyToTags(tags[s2t], funcName, tagName)
    #if funcName not in tags[s2t]:
    #  tags[s2t][funcName] = []
    #if tagName not in tags[s2t][funcName]:
    #  tags[s2t][funcName].append(tagName)

    for tag,preds in predicateTags.items():
      for p in preds:
        if p(funcName, methodSignature):
          addKeyToTags(tags[t2s], tag, funcName)
          addKeyToTags(tags[s2t], funcName, tag)
          break
    
    """if isConstructorMethod(methodSignature):
      tags[s2t][funcName].append("constructor")
      if conTag not in tags[t2s]:
        tags[t2s][conTag] = []
      if funcName not in tags[t2s][conTag]:
        tags[t2s][conTag].append(funcName)
    if funcName.find("init") > -1:
      if initTag not in tags[s2t][funcName]:
        tags[s2t][funcName].append(initTag)
      if initTag not in tags[t2s]:
        tags[t2s][initTag] = []
      if funcName not in tags[t2s][initTag]:
        tags[t2s][initTag].append(funcName)
    if isRelatedTo(funcName, methodSignature, subTag):
      addKeyToTags(tags[t2s], subTag, funcName)
      addKeyToTags(tags[s2t], funcName, subTag)
    if isRelatedTo(funcName, methodSignature,otherSubTag):
      addKeyToTags(tags[t2s], subTag, funcName)
      addKeyToTags(tags[s2t], funcName, subTag)
    if isRelatedTo(funcName, methodSignature, pubTag):
      addKeyToTags(tags[t2s], pubTag, funcName)
      addKeyToTags(tags[s2t], funcName, pubTag)
    if isRelatedTo(funcName, methodSignature, otherConTag):
      addKeyToTags(tags[t2s], conTag, funcName)
      addKeyToTags(tags[s2t], funcName, conTag)"""


with open("tags.json", "w") as f:
  f.write(json.dumps(tags, indent=4))
